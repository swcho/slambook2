# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 11 — Bundle Adjustment
#
# `wasm-src/spike/verify_ba.ts` 의 4 케이스를 Python LM (scipy + sparse Jacobian) 으로 미러:
#
# - Stage 2 회귀 (noiseless P=3 L=20, 3 seeds): chi² → 1e-10 미만, geom 수렴
# - 1 px Gaussian noise P=4 L=40 (3 seeds): chi² 1e-3 비율로 감소
# - Adaptive chi² loop (chi²_init=0.05, 1 px noise): doublings ≥ 4, inlier_ratio > 0.5
# - Stereo (left+right ext) — scale gauge 고정 검증

# %%
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import cv2
import numpy as np

from myslam_ref.ba import optimize

# %% [markdown]
# ## 합성 BA 장면 생성 (verify_ba.ts §makeScene 미러)

# %%
def rng_seeded(seed: int):
    state = int(seed) & 0xFFFFFFFF

    def rand() -> float:
        nonlocal state
        state = (state + 0x6D2B79F5) & 0xFFFFFFFF
        t = state
        t = ((t ^ (t >> 15)) * (1 | t)) & 0xFFFFFFFF
        t = ((t + ((t ^ (t >> 7)) * (61 | t))) ^ t) & 0xFFFFFFFF
        return float((t ^ (t >> 14)) & 0xFFFFFFFF) / 4294967296.0

    return rand


def axang_to_R(ax, ay, az):
    rvec = np.array([ax, ay, az], dtype=np.float64).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    return R


def gauss(rand):
    u = max(rand(), 1e-12)
    v = rand()
    return float(np.sqrt(-2 * np.log(u)) * np.cos(2 * np.pi * v))


def make_scene(seed: int, P: int, L: int, noise_std_px: float):
    rand = rng_seeded(seed)
    uni = lambda: rand() * 2 - 1
    fx, fy, cx, cy = 520, 520, 320, 240
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    poses_gt = [(np.eye(3), np.zeros(3))]
    for _ in range(1, P):
        R = axang_to_R(uni() * 0.2, uni() * 0.2, uni() * 0.2)
        t = np.array([uni() * 0.3, uni() * 0.3, uni() * 0.3])
        poses_gt.append((R, t))
    landmarks_gt = np.array([[uni(), uni(), 2 + rand() * 1.5] for _ in range(L)])

    obs = []
    for p in range(P):
        R, t = poses_gt[p]
        for li in range(L):
            Pw = landmarks_gt[li]
            Pc = R @ Pw + t
            if Pc[2] <= 0.5:
                continue
            u = fx * Pc[0] / Pc[2] + cx + (gauss(rand) * noise_std_px if noise_std_px else 0)
            v = fy * Pc[1] / Pc[2] + cy + (gauss(rand) * noise_std_px if noise_std_px else 0)
            obs.append([p, li, u, v, 1])

    init_poses = np.zeros((P, 12))
    for p in range(P):
        R_gt, t_gt = poses_gt[p]
        rvec_gt = cv2.Rodrigues(R_gt)[0].reshape(-1)
        pert = 0 if p == 0 else 0.15
        rinit = rvec_gt + pert * np.array([uni(), uni(), uni()])
        tinit = t_gt + pert * np.array([uni(), uni(), uni()])
        R_init = axang_to_R(*rinit)
        init_poses[p, :3] = R_init[0]; init_poses[p, 3] = tinit[0]
        init_poses[p, 4:7] = R_init[1]; init_poses[p, 7] = tinit[1]
        init_poses[p, 8:11] = R_init[2]; init_poses[p, 11] = tinit[2]

    init_lms = landmarks_gt + 0.2 * (np.array([[uni(), uni(), uni()] for _ in range(L)]))
    return dict(
        K=K, poses_gt=poses_gt, landmarks_gt=landmarks_gt,
        observations=np.asarray(obs, dtype=np.float64),
        init_poses=init_poses, init_lms=init_lms,
    )


IDENTITY_EXT = np.hstack([np.eye(3), np.zeros((3, 1))])

# %% [markdown]
# ## (1) Stage 2 회귀: noiseless P=3 L=20

# %%
print("[Step 11] Stage 2 회귀: noiseless P=3 L=20")
all_pass = True
for s in range(1, 4):
    sc = make_scene(s, 3, 20, 0)
    res = optimize(sc["init_poses"], sc["init_lms"], sc["observations"],
                   np.array([0]), sc["K"], IDENTITY_EXT, IDENTITY_EXT,
                   iterations=20, chi2_init=5.991, adaptive_rounds=5, use_robust_kernel=False)
    max_rot, max_tr, max_lm = 0.0, 0.0, 0.0
    for p in range(3):
        R_est = res.refined_poses12[p].reshape(3, 4)[:, :3]
        t_est = res.refined_poses12[p].reshape(3, 4)[:, 3]
        R_gt, t_gt = sc["poses_gt"][p]
        rvec, _ = cv2.Rodrigues(R_est @ R_gt.T)
        max_rot = max(max_rot, float(np.linalg.norm(rvec)))
        max_tr = max(max_tr, float(np.linalg.norm(t_est - t_gt)))
    for l in range(20):
        max_lm = max(max_lm, float(np.linalg.norm(res.refined_landmarks3[l] - sc["landmarks_gt"][l])))
    chi2_pass = res.final_chi2_sum < 1e-8  # noiseless: virtually zero
    geom_pass = max_rot < 5e-3 and max_tr < 5e-2 and max_lm < 1e-1
    ok = chi2_pass and geom_pass
    print(f"  [{'PASS' if ok else 'FAIL'}] seed={s}: chi2 {res.initial_chi2_sum:.2e} → {res.final_chi2_sum:.2e}  maxRot={max_rot:.2e} maxTr={max_tr:.2e} maxLm={max_lm:.2e}")
    all_pass &= ok

# %% [markdown]
# ## (2) 1 px Gaussian noise P=4 L=40

# %%
print("\n[Step 11] 1 px Gaussian noise P=4 L=40")
for s in range(10, 13):
    sc = make_scene(s, 4, 40, 1.0)
    res = optimize(sc["init_poses"], sc["init_lms"], sc["observations"],
                   np.array([0]), sc["K"], IDENTITY_EXT, IDENTITY_EXT,
                   iterations=20, chi2_init=5.991, adaptive_rounds=5, use_robust_kernel=False)
    max_rot, max_tr, max_lm = 0.0, 0.0, 0.0
    for p in range(4):
        R_est = res.refined_poses12[p].reshape(3, 4)[:, :3]
        t_est = res.refined_poses12[p].reshape(3, 4)[:, 3]
        R_gt, t_gt = sc["poses_gt"][p]
        rvec, _ = cv2.Rodrigues(R_est @ R_gt.T)
        max_rot = max(max_rot, float(np.linalg.norm(rvec)))
        max_tr = max(max_tr, float(np.linalg.norm(t_est - t_gt)))
    for l in range(40):
        max_lm = max(max_lm, float(np.linalg.norm(res.refined_landmarks3[l] - sc["landmarks_gt"][l])))
    chi2_pass = res.final_chi2_sum < res.initial_chi2_sum * 1e-2
    geom_pass = max_rot < 5e-2 and max_tr < 5e-2 and max_lm < 2e-1
    ok = chi2_pass and geom_pass
    print(f"  [{'PASS' if ok else 'FAIL'}] seed={s}: chi2 {res.initial_chi2_sum:.2e} → {res.final_chi2_sum:.2e}  maxRot={max_rot:.2e} maxTr={max_tr:.2e} maxLm={max_lm:.2e}")
    all_pass &= ok

# %% [markdown]
# ## (3) Adaptive chi² loop — chi²_init=0.05, 1 px noise

# %%
print("\n[Step 11] adaptive chi² loop (chi²_init=0.05, 1 px noise)")
sc = make_scene(31, 3, 30, 1.0)
res = optimize(sc["init_poses"], sc["init_lms"], sc["observations"],
               np.array([0]), sc["K"], IDENTITY_EXT, IDENTITY_EXT,
               iterations=15, chi2_init=0.05, adaptive_rounds=8, use_robust_kernel=False)
ok = res.adaptive_doublings >= 4 and res.final_inlier_ratio > 0.5 and np.isfinite(res.final_chi2_sum)
print(f"  [{'PASS' if ok else 'FAIL'}] doublings={res.adaptive_doublings} finalThr={res.final_chi2_threshold:.2f} inlierRatio={res.final_inlier_ratio:.3f} ({res.final_inlier_count}/{res.O})")
all_pass &= ok

# %% [markdown]
# ## (4) Stereo (left+right ext) — scale gauge 고정

# %%
print("\n[Step 11] stereo (left+right ext)")
sc = make_scene(7, 3, 25, 0)
right_t = -0.5
fx, fy, cx, cy = 520, 520, 320, 240
extra = []
for p in range(3):
    R, t = sc["poses_gt"][p]
    for li in range(25):
        Pw = sc["landmarks_gt"][li]
        Pc = R @ Pw + t + np.array([right_t, 0, 0])
        if Pc[2] <= 0.5:
            continue
        u = fx * Pc[0] / Pc[2] + cx
        v = fy * Pc[1] / Pc[2] + cy
        extra.append([p, li, u, v, 0])
obs_stereo = np.vstack([sc["observations"], np.asarray(extra)])
RIGHT_EXT = np.array([[1, 0, 0, right_t], [0, 1, 0, 0], [0, 0, 1, 0]], dtype=np.float64)
res = optimize(sc["init_poses"], sc["init_lms"], obs_stereo,
               np.array([0]), sc["K"], IDENTITY_EXT, RIGHT_EXT,
               iterations=20, chi2_init=5.991, adaptive_rounds=5, use_robust_kernel=False)
max_lm = max(float(np.linalg.norm(res.refined_landmarks3[l] - sc["landmarks_gt"][l])) for l in range(25))
ok = res.final_chi2_sum < 1e-6 and max_lm < 1e-3 and obs_stereo.shape[0] > sc["observations"].shape[0]
print(f"  [{'PASS' if ok else 'FAIL'}] leftObs={sc['observations'].shape[0]} totalObs={obs_stereo.shape[0]} chi2 {res.initial_chi2_sum:.2e} → {res.final_chi2_sum:.2e} maxLm={max_lm:.2e}")
all_pass &= ok

# %%
assert all_pass, "step11 BA gate failed — see above"
print("\nOK — step11 BA: LM + Schur (scipy sparse) + adaptive chi² + stereo ext all match verify_ba.ts")
