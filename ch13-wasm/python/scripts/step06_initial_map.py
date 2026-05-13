# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 06 — Initial map (Detect → Stereo LK → Triangulate)
#
# Step 3 → 4 → 5 의 파이프라인 합성을 KITTI mini frame 0 에 적용.
# ch13 `Frontend::StereoInit` 와 같은 흐름:
#
# 1. left 에서 GFTT 시드 (≥ ``num_features_init = 50``)
# 2. left → right LK (rectified stereo)
# 3. 트래킹된 페어로 triangulate (depth > 0 + σ4/σ3 < threshold)
# 4. accepted landmarks 가 ≥ 30 (책 기본값과 같은 게이트)

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
from myslam_ref.triangulation import Algo, triangulate
from myslam_ref.viz import draw_keypoints, draw_pointcloud_3d  # noqa: F401

# %%
ds = load_kitti_mini()
cam0 = ds.camera0
cam1 = ds.camera1

frame0 = ds.frames[0]
left, right = frame0.left, frame0.right
print(f"frame 0: {left.shape}; baseline {cam1.baseline_m:.3f} m, fx {cam0.fx:.2f}")

# %% [markdown]
# ## 1. Detect

# %%
NUM_FEATURES_INIT = 50  # ch13 frontend.cpp 기본값
seeds = detect(left, Detector.GFTT, maxFeatures=200, qualityLevel=0.01, minDistance=20)
print(f"detected {seeds.shape[0]} keypoints")
assert seeds.shape[0] >= NUM_FEATURES_INIT

# %% [markdown]
# ## 2. Stereo LK

# %%
tracked = track_lk(left, right, seeds, winSize=11, maxLevel=3, maxIter=30, eps=0.01)
ok_mask = tracked[:, 2] > 0.5
tracked_count = int(ok_mask.sum())
track_rate = tracked_count / seeds.shape[0]
print(f"stereo LK tracked {tracked_count}/{seeds.shape[0]} ({track_rate * 100:.1f}%)")
assert track_rate >= 0.5

# %% [markdown]
# ## 3. Triangulate (LinearSVD)
#
# left 카메라 = world frame, right 카메라는 ``t = (-baseline, 0, 0)`` 이동.

# %%
k_arr = np.array([cam0.fx, cam0.fy, cam0.cx, cam0.cy])
T_l = np.hstack([np.eye(3), np.zeros((3, 1))])
T_r = np.hstack([np.eye(3), np.array([[-cam1.baseline_m], [0], [0]])])
pts3d = triangulate(seeds, tracked, k_arr, T_l, k_arr, T_r, algo=Algo.LINEAR_SVD, quality_threshold=0.01)
accepted = pts3d[pts3d[:, 4] > 0.5]
print(f"triangulated → accepted {accepted.shape[0]} (depth>0 + σ4/σ3 < 0.01)")
assert accepted.shape[0] >= 30, f"need ≥ 30 landmarks, got {accepted.shape[0]}"

# Depth range sanity — KITTI 도로 장면이면 5 m ~ 50 m 사이 정도.
depths = accepted[:, 2]
print(f"accepted depth range: {depths.min():.1f} m ~ {depths.max():.1f} m, median {np.median(depths):.1f} m")
assert depths.min() > 0
assert np.median(depths) < 100, "median depth surprisingly large"

# %% [markdown]
# ## 4. 시각화 — left frame 의 seed overlay + 초기 landmark cloud

# %%
import matplotlib.pyplot as plt

fig_kp, ax_kp = plt.subplots(figsize=(12, 4))
draw_keypoints(left, seeds, ax=ax_kp, title=f"GFTT seeds on KITTI mini frame 0 left ({seeds.shape[0]} kps)")

# %%
fig_pc = draw_pointcloud_3d(
    accepted[:, :3],
    title=f"Initial landmark cloud ({accepted.shape[0]} points) — KITTI mini frame 0",
)

# %%
print(f"OK — step06 initial map: {accepted.shape[0]} landmarks from KITTI mini frame 0")
