# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 08 — Pose-only PnP (4-round outlier loop)
#
# `wasm-src/spike/verify_pnp.ts` 의 4 가지 시나리오를 미러:
#
# (A) Noiseless N=40 × 5 seeds → rotErr / trErr ≈ machine precision
# (B) 1 px Gaussian noise N=80 × 3 seeds → rotErr < 1e-2 rad, trErr < 5e-2
# (C) 20 % seeded outliers N=100 × 3 seeds → recall ≥ 80 %, rotErr < 5e-2
# (D) RobustKernel 옵션 smoke

# %%
from dataclasses import dataclass

import matplotlib.pyplot as plt
import cv2
import numpy as np

from myslam_ref.pnp import estimate_pose
from myslam_ref.viz import draw_residual_histogram  # noqa: F401

# %% [markdown]
# ## 합성 PnP 케이스 생성 (verify_pnp.ts §makeCase 미러)

# %%


@dataclass
class PnPCase:
    K: np.ndarray             # 3x3 카메라 내참수
    gt_world_pts3: np.ndarray # (N, 3) ground-truth world points
    uv_obs_pts2: np.ndarray   # (N, 2) 노이즈/아웃라이어가 섞인 관측 (u, v)
    R_gt: np.ndarray          # 3x3 ground-truth rotation (world→camera)
    t_gt: np.ndarray          # (3,) ground-truth translation
    init_pose6: np.ndarray    # (6,) [t | rvec] 초기 추정 (정답에 perturb 가한 것)
    outlier_idx: list[int]    # outlier 로 만든 인덱스


def rng_seeded(seed: int):
    # Mulberry32 — verify_pnp.ts 와 동일. uint32 wrap-around 를 의도적으로 사용.
    state = int(seed) & 0xFFFFFFFF

    def rand() -> float:
        nonlocal state
        state = (state + 0x6D2B79F5) & 0xFFFFFFFF
        t = state
        t = ((t ^ (t >> 15)) * (1 | t)) & 0xFFFFFFFF
        t = ((t + ((t ^ (t >> 7)) * (61 | t))) ^ t) & 0xFFFFFFFF
        return float((t ^ (t >> 14)) & 0xFFFFFFFF) / 4294967296.0

    return rand


def make_case(seed: int, N: int, noise_std_px: float, outlier_fraction: float = 0.0, outlier_shift: float = 60.0):
    rand = rng_seeded(seed)
    def uni(): return rand() * 2 - 1  # 0 < ... < 1을 -1 < ... < 1로 변경

    # rotation vector를 회전 행렬 R로 반환
    def axang_to_R(ax, ay, az):
        rvec = np.array([ax, ay, az], dtype=np.float64).reshape(3, 1)
        R, _ = cv2.Rodrigues(rvec)
        return R

    #  0.3 을 곱하는 이유:
    #  1. 프레임 간 모션 시뮬레이션 — Step 08 은 SLAM 의 PnP 단계(연속 프레임에서 카메라 자세 추정)를 검증합니다. 실제 비디오에서 인접 프레임 간 회전은 보통 몇 도 ~ 수십 도 수준이므로 최대 ~30° 가 현실적인 상한.
    #  2. 점들이 카메라 뒤로 가는 것을 방지 — line 73 에서 if Pc[2] <= 0.5: continue 로 카메라 뒤(또는 너무 가까운) 점을 거부합니다. 회전이 크면 reject 비율이 폭증해 make_case 가 N 개 채우는 데 매우 오래 걸리거나 점 분포가 편향됨. 0.3 정도면 reject 가 거의 없음.
    #  3. PnP 솔버의 수렴 보장 — solvePnP (특히 iterative / EPnP) 는 비선형 최적화라 초기값 근처 local minima 에 빠질 수 있음. 시나리오 A (noiseless) 에서 rotErr ≈ machine precision 까지 떨어져야 검증이 통과되는데, 회전이 너무 크면 솔버가 다른 해로 빠질 위험이 있음.
    #  4. 검증 시나리오의 정상 작동 보장 — line 14–17 에 명시된 통과 기준 (rotErr < 1e-2, recall ≥ 80% 등) 은 "합리적인 자세 범위" 를 가정. 0.3 은 그 범위를 적당히 좁게 잡아 테스트가 결정론적으로 통과하게 만드는 값.
    #  5. TS 레퍼런스와 bit-exact 일치 — 가장 본질적인 이유: verify_pnp.ts 의 makeCase 가 동일하게 uni()*0.3 을 쓰므로, Python 측에서 같은 시드로 같은 R_gt 가 나와야 결과를 1:1 로 비교할 수 있음 (앞서 설명한 [[Mulberry32 PRNG]] 동기화와 같은 맥락).
    #  참고로 t_gt 도 같은 철학:
    #  - x, y: uni() * 0.2 → ±0.2 m (좌우/상하 작은 이동)
    #  - z: 2 + rand() * 0.5 → [2.0, 2.5) m (항상 카메라 앞, 적당한 거리)
    R_gt = axang_to_R(uni() * 0.3, uni() * 0.3, uni() * 0.3)
    t_gt = np.array([uni() * 0.2, uni() * 0.2, 2 + rand() * 0.5])

    fx, fy, cx, cy = 520, 520, 320, 240
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    def gauss():
        u = max(rand(), 1e-12)
        v = rand()
        return float(np.sqrt(-2 * np.log(u)) * np.cos(2 * np.pi * v))

    gt_world_pts3: list[np.ndarray] = []  # world point([x, y, z]) 목록
    uv_obs_pts2: list[list[float]] = []   # 각 world point에 대한 [u, v]
    outlier_idx: list[int] = []
    i = 0
    while i < N:
        # world point 생성
        Pw = np.array([uni() * 1.5, uni() * 1.5, uni() * 1.5])
        Pc = R_gt @ Pw + t_gt  # camera point 계산
        if Pc[2] <= 0.5:
            continue

        nu = gauss() * noise_std_px if noise_std_px else 0
        nv = gauss() * noise_std_px if noise_std_px else 0
        u = fx * Pc[0] / Pc[2] + cx + nu  # gaussian noise 추가하여 카메라 이미지 좌표 계산
        v = fy * Pc[1] / Pc[2] + cy + nv

        # 파라미터로 받은 확률로 outlier 생성 여부 결정
        is_outlier = outlier_fraction > 0 and rand() < outlier_fraction
        if is_outlier:  # outlier일 경우 u, v 값 변경
            u += (1 if uni() >= 0 else -1) * outlier_shift
            v += (1 if uni() >= 0 else -1) * outlier_shift
            outlier_idx.append(i)
        gt_world_pts3.append(Pw)
        uv_obs_pts2.append([u, v])
        i += 1

    R_gt_rvec, _ = cv2.Rodrigues(R_gt)
    init_pose6 = np.concatenate([
        t_gt + np.array([uni() * 0.3, uni() * 0.3, uni() * 0.3]),
        R_gt_rvec.reshape(-1) +
        np.array([uni() * 0.1, uni() * 0.1, uni() * 0.1]),
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
# ## (A) Noiseless × 5 seeds, N=40 — machine precision

# %%
all_pass = True
for s in range(1, 6):
    d = make_case(seed=s, N=40, noise_std_px=0)
    res = estimate_pose(d.gt_world_pts3, d.uv_obs_pts2, d.K, d.init_pose6,
                        rounds=4, iter_per_round=10, chi2_threshold=5.991)
    R_est = res.T_cw[:, :3]
    t_est = res.T_cw[:, 3]
    rot_err, t_err = measure(R_est, t_est, d.R_gt, d.t_gt)
    status = "PASS" if rot_err < 1e-6 and t_err < 1e-6 else "FAIL"
    print(f"  [{status}] seed={s}: rotErr={rot_err:.2e} trErr={t_err:.2e} inliers={res.total_inliers}/{res.N}")
    all_pass &= rot_err < 1e-6 and t_err < 1e-6
    all_pass &= res.total_inliers == res.N

# %% [markdown]
# ## (B) 1 px Gaussian noise × 3 seeds, N=80 — mrad / cm

# %%
for s in range(10, 13):
    d = make_case(s, 80, 1.0)
    res = estimate_pose(d.gt_world_pts3, d.uv_obs_pts2, d.K, d.init_pose6,
                        rounds=4, iter_per_round=10, chi2_threshold=5.991)
    rot_err, t_err = measure(
        res.T_cw[:, :3], res.T_cw[:, 3], d.R_gt, d.t_gt)
    status = "PASS" if rot_err < 1e-2 and t_err < 5e-2 else "FAIL"
    print(f"  [{status}] seed={s}: rotErr={rot_err:.2e} trErr={t_err:.2e} inliers={res.total_inliers}/{res.N}")
    all_pass &= rot_err < 1e-2 and t_err < 5e-2

# %% [markdown]
# ## (C) 20 % seeded outliers × 3 seeds — recall ≥ 80 %

# %%
for s in range(20, 23):
    d = make_case(s, 100, 1.0, outlier_fraction=0.2, outlier_shift=60)
    res = estimate_pose(d.gt_world_pts3, d.uv_obs_pts2, d.K, d.init_pose6,
                        rounds=4, iter_per_round=10, chi2_threshold=5.991)
    rot_err, t_err = measure(
        res.T_cw[:, :3], res.T_cw[:, 3], d.R_gt, d.t_gt)
    hits = sum(1 for idx in d.outlier_idx
               if res.final_inlier_mask[idx] == 0)
    recall = hits / len(d.outlier_idx) if d.outlier_idx else 1.0
    status = "PASS" if (rot_err < 5e-2 and t_err <
                        1e-1 and recall >= 0.8) else "FAIL"
    print(
        f"  [{status}] seed={s}: rotErr={rot_err:.2e} trErr={t_err:.2e} "
        f"recall={hits}/{len(d.outlier_idx)} ({recall * 100:.0f}%) inliers={res.total_inliers}/{res.N}"
    )
    all_pass &= rot_err < 5e-2 and t_err < 1e-1 and recall >= 0.8

# %% [markdown]
# ## (D) RobustKernel toggle smoke

# %%
d = make_case(30, 60, 1.0)
for label, opts in [
    ("kernel=on, drop=4 (never drop)", dict(rounds=4, remove_robust_after_round=4)),
    ("kernel=off entirely",            dict(rounds=4, use_robust_kernel=False)),
    ("kernel=on, drop=0 (book extreme)", dict(
        rounds=4, remove_robust_after_round=0)),
]:
    res = estimate_pose(d.gt_world_pts3, d.uv_obs_pts2, d.K, d.init_pose6,
                        iter_per_round=10, chi2_threshold=5.991, **opts)
    rot_err, t_err = measure(
        res.T_cw[:, :3], res.T_cw[:, 3], d.R_gt, d.t_gt)
    status = "PASS" if (rot_err < 5e-2 and t_err < 1e-1) else "FAIL"
    print(f"  [{status}] {label}: rotErr={rot_err:.2e} trErr={t_err:.2e}")
    all_pass &= rot_err < 5e-2 and t_err < 1e-1

# %%
assert all_pass, "step08 PnP gate failed — see above"

# %% [markdown]
# ## 5. 시각화 — outlier 케이스에서 chi² 분포

# %%

d_viz = make_case(20, 100, 1.0, outlier_fraction=0.2, outlier_shift=60)
res_viz = estimate_pose(d_viz.gt_world_pts3, d_viz.uv_obs_pts2, d_viz.K,
                        d_viz.init_pose6, rounds=4, iter_per_round=10, chi2_threshold=5.991)
proj, _ = cv2.projectPoints(d_viz.gt_world_pts3, cv2.Rodrigues(
    res_viz.T_cw[:, :3])[0], res_viz.T_cw[:, 3], d_viz.K, None)
residuals_sq = np.sum((d_viz.uv_obs_pts2 - proj.reshape(-1, 2)) ** 2, axis=1)
fig_r = draw_residual_histogram(
    residuals_sq,
    chi2_threshold=5.991,
    title=f"seed=20, 20% outliers · final chi² (n_inlier={res_viz.total_inliers}/{res_viz.N})",
)

# %%
# 4 round inlier count 추세 — 라운드별로 inlier 가 어떻게 변하는지.
fig_ic, ax_ic = plt.subplots(figsize=(7, 3))
ax_ic.bar(range(1, len(res_viz.round_inlier_count) + 1),
          res_viz.round_inlier_count, color="steelblue")
ax_ic.set_xlabel("round")
ax_ic.set_ylabel("inlier count")
ax_ic.set_title(
    f"4-round outlier loop · seeded outliers {len(d_viz.outlier_idx)}, recall {sum(1 for i in d_viz.outlier_idx if res_viz.final_inlier_mask[i] == 0)}")
fig_ic.tight_layout()

# %%
print("OK — step08 PnP 4-round outlier loop matches verify_pnp.ts gate")
