# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 04 — Stereo matching (LK left → right)
#
# `bind_features.cpp` 의 ``trackLK`` 가 Step 03 에서 검증되었으므로 여기서는
# rectified KITTI mini stereo pair 에서 의미적 성질을 추가로 확인한다:
#
# - 수평 stereo 이므로 매칭 후 ``mean dy ≈ 0``
# - ``mean dx < 0`` (left → right 시차는 음수)
# - ``useInitialFlow`` 로 ``-baseline·fx/median_depth`` 초기 추정을 주면 트래킹율이
#   유의하게 향상

# %%
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np

from myslam_ref.dataset import load_kitti_mini
from myslam_ref.features import Detector, detect, track_lk
from myslam_ref.viz import draw_flow_field  # noqa: F401

# %%
ds = load_kitti_mini()
cam = ds.camera0
frame0 = ds.frames[0]
left = frame0.left
right = frame0.right
print(f"frame 0: left {left.shape} right {right.shape}")
print(f"baseline = {ds.camera1.baseline_m:.6f} m, fx = {cam.fx:.2f}")

# %% [markdown]
# ## 1. GFTT 시드 + LK (no init)

# %%
seeds = detect(left, Detector.GFTT, maxFeatures=200, qualityLevel=0.01, minDistance=20)
print(f"GFTT seeds on left: {seeds.shape[0]}")
tracked = track_lk(left, right, seeds, winSize=11, maxLevel=3, maxIter=30, eps=0.01)

ok = tracked[:, 2] > 0.5
n_ok = int(ok.sum())
n = seeds.shape[0]
dxs = tracked[ok, 0] - seeds[ok, 0]
dys = tracked[ok, 1] - seeds[ok, 1]
print(f"tracked {n_ok}/{n} ({n_ok / n * 100:.1f}%)")
print(f"mean dx = {dxs.mean():+.2f} px, mean dy = {dys.mean():+.2f} px")
print(f"dx percentiles: 25% {np.percentile(dxs, 25):.1f}, 50% {np.percentile(dxs, 50):.1f}, 75% {np.percentile(dxs, 75):.1f}")

assert n_ok >= int(0.7 * n), f"trackRate {n_ok / n:.1%} < 70%"
assert dxs.mean() < 0, f"mean dx should be negative (left→right shift), got {dxs.mean():.2f}"
assert abs(dys.mean()) < 1.5, f"mean dy should be near 0 for rectified pair, got {dys.mean():.2f}"

# %% [markdown]
# ## 2. ``useInitialFlow`` 로 초기 추정 주기
#
# 예상 disparity ≈ fx·baseline/Z. KITTI 도로 장면 평균 depth 가 ~15 m 라 가정.

# %%
expected_disparity = cam.fx * ds.camera1.baseline_m / 15.0
init_pts = seeds.copy()
init_pts[:, 0] -= expected_disparity
init_pts[:, 2] = 1.0
tracked_init = track_lk(
    left, right, seeds,
    winSize=11, maxLevel=3, maxIter=30, eps=0.01,
    useInitialFlow=True, initialPts=init_pts,
)
ok_i = tracked_init[:, 2] > 0.5
print(f"with init guess (expected disparity ≈ {expected_disparity:.1f} px): tracked {int(ok_i.sum())}/{n}")
assert int(ok_i.sum()) >= n_ok - 2  # tolerate ±2 jitter

# %% [markdown]
# ## 3. 시각화 — KITTI mini frame 0 의 stereo disparity flow

# %%
import matplotlib.pyplot as plt

fig_flow = draw_flow_field(
    left, seeds[:, :2], tracked[:, :2], status=tracked[:, 2],
    title=f"KITTI mini frame 0: left → right LK ({n_ok}/{n} tracked, mean dx={dxs.mean():+.1f} px)",
    step=2,
)

# %%
# Disparity 분포 — depth 로 변환하면 ``Z = fx · baseline / disparity``.
fig_d, ax_d = plt.subplots(figsize=(8, 3))
disp = -dxs  # left → right shift 의 절댓값
ax_d.hist(disp, bins=30, color="steelblue", alpha=0.8)
ax_d.set_xlabel("disparity (px)")
ax_d.set_ylabel("count")
ax_d.set_title(f"KITTI mini frame 0 stereo disparity (median {float(np.median(disp)):.1f} px → ~{cam.fx * ds.camera1.baseline_m / float(np.median(disp)):.1f} m depth)")
fig_d.tight_layout()

# %%
print("OK — step04 stereo matching: rectified-pair LK has near-zero dy and negative dx")
