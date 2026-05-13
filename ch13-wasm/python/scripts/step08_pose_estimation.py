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
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import cv2
import numpy as np

from myslam_ref.pnp import estimate_pose

# %% [markdown]
# ## 합성 PnP 케이스 생성 (verify_pnp.ts §makeCase 미러)

# %%
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
    uni = lambda: rand() * 2 - 1

    def axang_to_R(ax, ay, az):
        rvec = np.array([ax, ay, az], dtype=np.float64).reshape(3, 1)
        R, _ = cv2.Rodrigues(rvec)
        return R

    R_gt = axang_to_R(uni() * 0.3, uni() * 0.3, uni() * 0.3)
    t_gt = np.array([uni() * 0.2, uni() * 0.2, 2 + rand() * 0.5])

    fx, fy, cx, cy = 520, 520, 320, 240
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    def gauss():
        u = max(rand(), 1e-12)
        v = rand()
        return float(np.sqrt(-2 * np.log(u)) * np.cos(2 * np.pi * v))

    pts3 = []
    pts2 = []
    outlier_idx = []
    i = 0
    while i < N:
        Pw = np.array([uni() * 1.5, uni() * 1.5, uni() * 1.5])
        Pc = R_gt @ Pw + t_gt
        if Pc[2] <= 0.5:
            continue
        is_outlier = outlier_fraction > 0 and rand() < outlier_fraction
        nu = gauss() * noise_std_px if noise_std_px else 0
        nv = gauss() * noise_std_px if noise_std_px else 0
        u = fx * Pc[0] / Pc[2] + cx + nu
        v = fy * Pc[1] / Pc[2] + cy + nv
        if is_outlier:
            u += (1 if uni() >= 0 else -1) * outlier_shift
            v += (1 if uni() >= 0 else -1) * outlier_shift
            outlier_idx.append(i)
        pts3.append(Pw)
        pts2.append([u, v])
        i += 1

    R_gt_rvec, _ = cv2.Rodrigues(R_gt)
    init_pose6 = np.concatenate([
        t_gt + np.array([uni() * 0.3, uni() * 0.3, uni() * 0.3]),
        R_gt_rvec.reshape(-1) + np.array([uni() * 0.1, uni() * 0.1, uni() * 0.1]),
    ])
    return dict(K=K, pts3=np.asarray(pts3), pts2=np.asarray(pts2), R_gt=R_gt, t_gt=t_gt, init_pose6=init_pose6, outlier_idx=outlier_idx)


def measure(R_est, t_est, R_gt, t_gt):
    R_err, _ = cv2.Rodrigues(R_est @ R_gt.T)
    return float(np.linalg.norm(R_err)), float(np.linalg.norm(t_est - t_gt))


# %% [markdown]
# ## (A) Noiseless × 5 seeds, N=40 — machine precision

# %%
all_pass = True
for s in range(1, 6):
    d = make_case(s, 40, 0)
    res = estimate_pose(d["pts3"], d["pts2"], d["K"], d["init_pose6"], rounds=4, iter_per_round=10, chi2_threshold=5.991)
    R_est = res.T_cw[:, :3]
    t_est = res.T_cw[:, 3]
    rot_err, t_err = measure(R_est, t_est, d["R_gt"], d["t_gt"])
    status = "PASS" if rot_err < 1e-6 and t_err < 1e-6 else "FAIL"
    print(f"  [{status}] seed={s}: rotErr={rot_err:.2e} trErr={t_err:.2e} inliers={res.total_inliers}/{res.N}")
    all_pass &= rot_err < 1e-6 and t_err < 1e-6
    all_pass &= res.total_inliers == res.N

# %% [markdown]
# ## (B) 1 px Gaussian noise × 3 seeds, N=80 — mrad / cm

# %%
for s in range(10, 13):
    d = make_case(s, 80, 1.0)
    res = estimate_pose(d["pts3"], d["pts2"], d["K"], d["init_pose6"], rounds=4, iter_per_round=10, chi2_threshold=5.991)
    rot_err, t_err = measure(res.T_cw[:, :3], res.T_cw[:, 3], d["R_gt"], d["t_gt"])
    status = "PASS" if rot_err < 1e-2 and t_err < 5e-2 else "FAIL"
    print(f"  [{status}] seed={s}: rotErr={rot_err:.2e} trErr={t_err:.2e} inliers={res.total_inliers}/{res.N}")
    all_pass &= rot_err < 1e-2 and t_err < 5e-2

# %% [markdown]
# ## (C) 20 % seeded outliers × 3 seeds — recall ≥ 80 %

# %%
for s in range(20, 23):
    d = make_case(s, 100, 1.0, outlier_fraction=0.2, outlier_shift=60)
    res = estimate_pose(d["pts3"], d["pts2"], d["K"], d["init_pose6"], rounds=4, iter_per_round=10, chi2_threshold=5.991)
    rot_err, t_err = measure(res.T_cw[:, :3], res.T_cw[:, 3], d["R_gt"], d["t_gt"])
    hits = sum(1 for idx in d["outlier_idx"] if res.final_inlier_mask[idx] == 0)
    recall = hits / len(d["outlier_idx"]) if d["outlier_idx"] else 1.0
    status = "PASS" if (rot_err < 5e-2 and t_err < 1e-1 and recall >= 0.8) else "FAIL"
    print(
        f"  [{status}] seed={s}: rotErr={rot_err:.2e} trErr={t_err:.2e} "
        f"recall={hits}/{len(d['outlier_idx'])} ({recall * 100:.0f}%) inliers={res.total_inliers}/{res.N}"
    )
    all_pass &= rot_err < 5e-2 and t_err < 1e-1 and recall >= 0.8

# %% [markdown]
# ## (D) RobustKernel toggle smoke

# %%
d = make_case(30, 60, 1.0)
for label, opts in [
    ("kernel=on, drop=4 (never drop)", dict(rounds=4, remove_robust_after_round=4)),
    ("kernel=off entirely",            dict(rounds=4, use_robust_kernel=False)),
    ("kernel=on, drop=0 (book extreme)", dict(rounds=4, remove_robust_after_round=0)),
]:
    res = estimate_pose(d["pts3"], d["pts2"], d["K"], d["init_pose6"], iter_per_round=10, chi2_threshold=5.991, **opts)
    rot_err, t_err = measure(res.T_cw[:, :3], res.T_cw[:, 3], d["R_gt"], d["t_gt"])
    status = "PASS" if (rot_err < 5e-2 and t_err < 1e-1) else "FAIL"
    print(f"  [{status}] {label}: rotErr={rot_err:.2e} trErr={t_err:.2e}")
    all_pass &= rot_err < 5e-2 and t_err < 1e-1

# %%
assert all_pass, "step08 PnP gate failed — see above"
print("OK — step08 PnP 4-round outlier loop matches verify_pnp.ts gate")
