# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 08 — Pose-only PnP · 4-way 비교
#
# `wasm-src/spike/verify_pnp.ts` 의 4 가지 시나리오를 미러하면서, inner LM
# 솔버를 **4 가지 변종으로 교차 비교**한다 (PLAN §3 Step 8 / TODO).
#
# | 변종 | inner LM | Huber 방식 |
# |---|---|---|
# | cv2         | ``cv2.solvePnP(SOLVEPNP_ITERATIVE)``         | mask freeze 근사 |
# | scipy       | ``scipy.optimize.least_squares(loss='huber')`` | scipy 내장 |
# | handwritten | 직접 작성한 SE(3) LM (``myslam_ref/pnp_lm.py``) | IRLS weight (g2o 동일) |
# | g2o         | ``gtsam.LevenbergMarquardtOptimizer``        | ``noiseModel.Robust(Huber)`` |
#
# 시나리오:
#
# (A) Noiseless N=40 × 5 seeds → rotErr / trErr ≈ machine precision
# (B) 1 px Gaussian noise N=80 × 3 seeds → rotErr < 1e-2 rad, trErr < 5e-2
# (C) 20 % seeded outliers N=100 × 3 seeds → recall ≥ 80 %, rotErr < 5e-2
# (D) RobustKernel 옵션 smoke

# %%
import time
from dataclasses import dataclass

import cv2
import matplotlib.pyplot as plt
import numpy as np

from myslam_ref.pnp import (
    estimate_pose_cv2,
    estimate_pose_g2o,
    estimate_pose_handwritten,
    estimate_pose_scipy,
)
from myslam_ref.viz import draw_residual_histogram


# %% [markdown]
# ## 합성 PnP 케이스 생성 (verify_pnp.ts §makeCase 미러)

# %%
@dataclass
class PnPCase:
    K: np.ndarray             # 3x3 카메라 내참수
    gt_world_pts3: np.ndarray  # (N, 3) ground-truth world points
    uv_obs_pts2: np.ndarray   # (N, 2) 노이즈/아웃라이어 섞인 관측 (u, v)
    R_gt: np.ndarray          # 3x3 ground-truth rotation (world→camera)
    t_gt: np.ndarray          # (3,) ground-truth translation
    init_pose6: np.ndarray    # (6,) [t | rvec] 초기 추정 (정답에 perturb)
    outlier_idx: list[int]    # outlier 로 만든 인덱스


def rng_seeded(seed: int):
    """Mulberry32 — verify_pnp.ts 와 동일. uint32 wrap-around 의도."""
    state = int(seed) & 0xFFFFFFFF

    def rand() -> float:
        nonlocal state
        state = (state + 0x6D2B79F5) & 0xFFFFFFFF
        t = state
        t = ((t ^ (t >> 15)) * (1 | t)) & 0xFFFFFFFF
        t = ((t + ((t ^ (t >> 7)) * (61 | t))) ^ t) & 0xFFFFFFFF
        return float((t ^ (t >> 14)) & 0xFFFFFFFF) / 4294967296.0

    return rand


def make_case(
    seed: int,
    N: int,
    noise_std_px: float,
    outlier_fraction: float = 0.0,
    outlier_shift: float = 60.0,
) -> PnPCase:
    rand = rng_seeded(seed)
    def uni(): return rand() * 2 - 1

    def axang_to_R(ax, ay, az):
        rvec = np.array([ax, ay, az], dtype=np.float64).reshape(3, 1)
        R, _ = cv2.Rodrigues(rvec)
        return R

    # 0.3 ≈ 17° 상한 — 인접 비디오 프레임 모션 가정 (자세한 사유는 git blame 참조).
    R_gt = axang_to_R(uni() * 0.3, uni() * 0.3, uni() * 0.3)
    t_gt = np.array([uni() * 0.2, uni() * 0.2, 2 + rand() * 0.5])

    fx, fy, cx, cy = 520, 520, 320, 240
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    def gauss():
        u = max(rand(), 1e-12)
        v = rand()
        return float(np.sqrt(-2 * np.log(u)) * np.cos(2 * np.pi * v))

    gt_world_pts3: list[np.ndarray] = []
    uv_obs_pts2: list[list[float]] = []
    outlier_idx: list[int] = []
    i = 0
    while i < N:
        Pw = np.array([uni() * 1.5, uni() * 1.5, uni() * 1.5])
        Pc = R_gt @ Pw + t_gt
        if Pc[2] <= 0.5:
            continue
        nu = gauss() * noise_std_px if noise_std_px else 0
        nv = gauss() * noise_std_px if noise_std_px else 0
        u = fx * Pc[0] / Pc[2] + cx + nu
        v = fy * Pc[1] / Pc[2] + cy + nv
        is_outlier = outlier_fraction > 0 and rand() < outlier_fraction
        if is_outlier:
            u += (1 if uni() >= 0 else -1) * outlier_shift
            v += (1 if uni() >= 0 else -1) * outlier_shift
            outlier_idx.append(i)
        gt_world_pts3.append(Pw)
        uv_obs_pts2.append([u, v])
        i += 1

    R_gt_rvec, _ = cv2.Rodrigues(R_gt)
    init_pose6 = np.concatenate([
        t_gt + np.array([uni() * 0.3, uni() * 0.3, uni() * 0.3]),
        R_gt_rvec.reshape(-1) + np.array([uni() * 0.1, uni() * 0.1, uni() * 0.1]),
    ])
    return PnPCase(
        K=K,
        gt_world_pts3=np.asarray(gt_world_pts3),
        uv_obs_pts2=np.asarray(uv_obs_pts2),
        R_gt=R_gt,
        t_gt=t_gt,
        init_pose6=init_pose6,
        outlier_idx=outlier_idx,
    )


def measure(R_est, t_est, R_gt, t_gt):
    R_err, _ = cv2.Rodrigues(R_est @ R_gt.T)
    return float(np.linalg.norm(R_err)), float(np.linalg.norm(t_est - t_gt))


# %% [markdown]
# ## 4 변종 dispatch 헬퍼
#
# ``estimate_pose_g2o`` 는 gtsam 미설치 환경에서 ``ImportError`` 를 던지므로
# 4-way 비교에서 그 한 행만 N/A 로 표기하고 나머지는 계속 비교한다.

# %%
VARIANTS = [
    ("cv2", estimate_pose_cv2),
    ("scipy", estimate_pose_scipy),
    ("handwritten", estimate_pose_handwritten),
    ("g2o", estimate_pose_g2o),
]


@dataclass
class GateRow:
    variant: str
    A_rot: float | None = None    # max rotErr over (A) seeds
    A_tr: float | None = None     # max trErr over (A) seeds
    B_rot: float | None = None
    B_tr: float | None = None
    C_rot: float | None = None
    C_tr: float | None = None
    C_recall: float | None = None
    D_pass: bool | None = None    # D 시나리오 통과 여부
    wallclock_s: float = 0.0
    available: bool = True
    skip_reason: str = ""


def run_gate(estimate_fn) -> GateRow:
    """A/B/C/D 한 묶음을 돌려 한 변종의 요약 행을 만든다."""
    row = GateRow(variant=estimate_fn.__name__.replace("estimate_pose_", ""))
    t0 = time.perf_counter()
    try:
        # (A) noiseless × 5 seeds → max error 누적
        a_rot, a_tr = 0.0, 0.0
        for s in range(1, 6):
            d = make_case(s, 40, 0)
            res = estimate_fn(
                d.gt_world_pts3, d.uv_obs_pts2, d.K, d.init_pose6,
                rounds=4, iter_per_round=10, chi2_threshold=5.991,
            )
            r, t = measure(res.T_cw[:, :3], res.T_cw[:, 3], d.R_gt, d.t_gt)
            a_rot, a_tr = max(a_rot, r), max(a_tr, t)
        row.A_rot, row.A_tr = a_rot, a_tr

        # (B) 1 px noise × 3 seeds
        b_rot, b_tr = 0.0, 0.0
        for s in range(10, 13):
            d = make_case(s, 80, 1.0)
            res = estimate_fn(d.gt_world_pts3, d.uv_obs_pts2, d.K, d.init_pose6,
                              rounds=4, iter_per_round=10, chi2_threshold=5.991)
            r, t = measure(res.T_cw[:, :3], res.T_cw[:, 3], d.R_gt, d.t_gt)
            b_rot, b_tr = max(b_rot, r), max(b_tr, t)
        row.B_rot, row.B_tr = b_rot, b_tr

        # (C) 20 % outliers × 3 seeds
        c_rot, c_tr, recalls = 0.0, 0.0, []
        for s in range(20, 23):
            d = make_case(s, 100, 1.0, outlier_fraction=0.2, outlier_shift=60)
            res = estimate_fn(d.gt_world_pts3, d.uv_obs_pts2, d.K, d.init_pose6,
                              rounds=4, iter_per_round=10, chi2_threshold=5.991)
            r, t = measure(res.T_cw[:, :3], res.T_cw[:, 3], d.R_gt, d.t_gt)
            hits = sum(1 for idx in d.outlier_idx if res.final_inlier_mask[idx] == 0)
            recalls.append(hits / max(1, len(d.outlier_idx)))
            c_rot, c_tr = max(c_rot, r), max(c_tr, t)
        row.C_rot, row.C_tr = c_rot, c_tr
        row.C_recall = float(min(recalls))

        # (D) kernel toggle smoke
        d = make_case(30, 60, 1.0)
        d_ok = True
        for opts in [
            dict(rounds=4, remove_robust_after_round=4),
            dict(rounds=4, use_robust_kernel=False),
            dict(rounds=4, remove_robust_after_round=0),
        ]:
            res = estimate_fn(d.gt_world_pts3, d.uv_obs_pts2, d.K, d.init_pose6,
                              iter_per_round=10, chi2_threshold=5.991, **opts)
            r, t = measure(res.T_cw[:, :3], res.T_cw[:, 3], d.R_gt, d.t_gt)
            d_ok &= r < 5e-2 and t < 1e-1
        row.D_pass = d_ok
    except ImportError as e:
        row.available = False
        row.skip_reason = str(e).splitlines()[0]
    finally:
        row.wallclock_s = time.perf_counter() - t0
    return row


# %% [markdown]
# ## 4 변종 동시 실행

# %%
rows: list[GateRow] = []
for name, fn in VARIANTS:
    rows.append(run_gate(fn))


def fmt(x, p=".2e"):
    return f"{x:{p}}" if x is not None else "  -- "


print(f"{'variant':12s} | {'A rotErr':>10s} | {'B rotErr':>10s} | {'C rotErr':>10s} | {'C recall':>10s} | {'D pass':>7s} | {'wallclock':>10s}")
print("-" * 86)
for r in rows:
    if not r.available:
        print(f"{r.variant:12s} | {'N/A':>10s} | {'N/A':>10s} | {'N/A':>10s} | {'N/A':>10s} | {'N/A':>7s} | {r.wallclock_s:8.3f} s   ({r.skip_reason[:40]})")
        continue
    recall = f"{(r.C_recall or 0) * 100:>9.0f}%"
    d_pass = "PASS" if r.D_pass else "FAIL"
    print(f"{r.variant:12s} | {fmt(r.A_rot):>10s} | {fmt(r.B_rot):>10s} | {fmt(r.C_rot):>10s} | {recall:>10s} | {d_pass:>7s} | {r.wallclock_s:8.3f} s")


# %% [markdown]
# ## 게이트 — 사용 가능한 변종들의 기준 통과 확인

# %%
def _gate_for(row: GateRow) -> tuple[bool, list[str]]:
    msgs: list[str] = []
    if not row.available:
        return True, [f"{row.variant}: skipped ({row.skip_reason[:60]})"]
    ok = True
    if row.A_rot is None or row.A_rot >= 1e-6 or (row.A_tr or 0) >= 1e-6:
        ok = False
        msgs.append(f"{row.variant} (A): rotErr {row.A_rot:.2e} / trErr {row.A_tr:.2e} not ≤ 1e-6")
    if row.B_rot is None or row.B_rot >= 1e-2 or (row.B_tr or 0) >= 5e-2:
        ok = False
        msgs.append(f"{row.variant} (B): rotErr {row.B_rot:.2e} / trErr {row.B_tr:.2e}")
    if row.C_rot is None or row.C_rot >= 5e-2 or (row.C_tr or 0) >= 1e-1 or (row.C_recall or 0) < 0.8:
        ok = False
        msgs.append(
            f"{row.variant} (C): rotErr {row.C_rot:.2e} / trErr {row.C_tr:.2e} / recall {(row.C_recall or 0) * 100:.0f}%"
        )
    if not row.D_pass:
        ok = False
        msgs.append(f"{row.variant} (D): kernel toggle smoke failed")
    return ok, msgs


all_pass = True
for r in rows:
    ok, msgs = _gate_for(r)
    for m in msgs:
        print(("  PASS " if ok else "  FAIL ") + m)
    all_pass &= ok

# Cross-check: 사용 가능한 변종들의 A 시나리오 결과가 서로 유사해야 한다.
# T_cw 자체를 비교하기보단 (A) seed=1 한 케이스를 골라 4 변종 모두 거의
# 동일한 R, t 를 내는지 확인.
d_xref = make_case(1, 40, 0.0)
T_refs = []
for name, fn in VARIANTS:
    try:
        res = fn(d_xref.gt_world_pts3, d_xref.uv_obs_pts2, d_xref.K, d_xref.init_pose6)
        T_refs.append((name, res.T_cw))
    except ImportError:
        pass
if len(T_refs) >= 2:
    base_name, base_T = T_refs[0]
    for name, T in T_refs[1:]:
        diff = float(np.linalg.norm(T - base_T))
        ok = diff < 1e-3
        all_pass &= ok
        print(("  PASS " if ok else "  FAIL ") + f"cross-check {name} vs {base_name}: ||ΔT||={diff:.2e}")


# %% [markdown]
# ## 4-panel 시각화 — 라운드별 inlier count

# %%
d_viz = make_case(20, 100, 1.0, outlier_fraction=0.2, outlier_shift=60)
viz_results: dict[str, object] = {}
for name, fn in VARIANTS:
    try:
        viz_results[name] = fn(
            d_viz.gt_world_pts3, d_viz.uv_obs_pts2, d_viz.K, d_viz.init_pose6,
            rounds=4, iter_per_round=10, chi2_threshold=5.991,
        )
    except ImportError:
        viz_results[name] = None

fig_ic, axes_ic = plt.subplots(1, 4, figsize=(14, 3), sharey=True)
for ax, (name, _fn) in zip(axes_ic, VARIANTS, strict=True):
    res = viz_results.get(name)
    if res is None:
        ax.text(0.5, 0.5, "N/A", ha="center", va="center", transform=ax.transAxes)
        ax.set_title(name)
        continue
    ax.bar(range(1, len(res.round_inlier_count) + 1),  # type: ignore[attr-defined]
           res.round_inlier_count, color="steelblue")  # type: ignore[attr-defined]
    ax.set_xlabel("round")
    ax.set_title(f"{name} · final {res.total_inliers}/{res.N}")  # type: ignore[attr-defined]
axes_ic[0].set_ylabel("inlier count")
fig_ic.suptitle(f"Round-by-round inliers · seed=20, 20 % seeded outliers ({len(d_viz.outlier_idx)})")
fig_ic.tight_layout()


# %% [markdown]
# ## 4-panel — final per-edge chi² histogram

# %%
fig_h, axes_h = plt.subplots(1, 4, figsize=(14, 3), sharey=True)
for ax, (name, _fn) in zip(axes_h, VARIANTS, strict=True):
    res = viz_results.get(name)
    if res is None:
        ax.text(0.5, 0.5, "N/A", ha="center", va="center", transform=ax.transAxes)
        ax.set_title(name)
        continue
    proj, _ = cv2.projectPoints(
        d_viz.gt_world_pts3,
        cv2.Rodrigues(res.T_cw[:, :3])[0],  # type: ignore[attr-defined]
        res.T_cw[:, 3],  # type: ignore[attr-defined]
        d_viz.K, None,
    )
    residuals_sq = np.sum((d_viz.uv_obs_pts2 - proj.reshape(-1, 2)) ** 2, axis=1)
    inlier_ct = int(res.total_inliers)  # type: ignore[attr-defined]
    # draw_residual_histogram 은 새 figure 를 만들기 때문에 ax 에 직접 그린다.
    ax.hist(residuals_sq[residuals_sq < 100], bins=30, color="steelblue", edgecolor="black")
    ax.axvline(5.991, color="crimson", linestyle="--", linewidth=1, label="χ² th")
    ax.set_xlabel("per-edge χ²")
    ax.set_title(f"{name} · in={inlier_ct}/{res.N}")  # type: ignore[attr-defined]
axes_h[0].set_ylabel("count")
fig_h.suptitle("Final per-edge χ² · seed=20, 20 % outliers (clipped to χ² < 100)")
fig_h.tight_layout()


# %% [markdown]
# ## 단일 (cv2) baseline 의 residual histogram (기존 viz.draw_residual_histogram)
# `cv2` 변종 결과를 기존 viz helper 로 한 번 더 그려서 비교 baseline 확보.

# %%
res_cv2 = viz_results.get("cv2")
if res_cv2 is not None:
    proj_cv2, _ = cv2.projectPoints(
        d_viz.gt_world_pts3,
        cv2.Rodrigues(res_cv2.T_cw[:, :3])[0],  # type: ignore[attr-defined]
        res_cv2.T_cw[:, 3],  # type: ignore[attr-defined]
        d_viz.K, None,
    )
    residuals_sq_cv2 = np.sum((d_viz.uv_obs_pts2 - proj_cv2.reshape(-1, 2)) ** 2, axis=1)
    draw_residual_histogram(
        residuals_sq_cv2,
        chi2_threshold=5.991,
        title=f"cv2 baseline · seed=20, 20 % outliers (in={res_cv2.total_inliers}/{res_cv2.N})",  # type: ignore[attr-defined]
    )


# %%
assert all_pass, "step08 PnP 4-way gate failed — see PASS/FAIL above"
print("OK — step08 PnP 4-way comparison gate matches verify_pnp.ts")
