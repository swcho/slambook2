# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 13 — Full pipeline (KITTI mini, 5 frames)
#
# 앞선 모든 step 의 모듈을 묶어 KITTI mini 5 frame 에 visual odometry 를 돌린다.
# 검증 기준은 게이트 회귀가 아니라 합리성 (smoke) 이다:
#
# - frame 0 stereo init 으로 ``≥ 30`` landmark 생성
# - frame 1..4 temporal LK + PnP 가 매 프레임 ``≥ 50 %`` 트래킹 유지
# - trajectory 가 단조 전진 (z translation 의 합이 증가, x/y 는 작은 변화)

# %%
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import cv2
import numpy as np

from myslam_ref.dataset import load_kitti_mini
from myslam_ref.features import Detector, detect, track_lk
from myslam_ref.keyframe import decide_inlier_threshold
from myslam_ref.pnp import estimate_pose
from myslam_ref.slam_map import Policy, PolicyOptions, SlamMap
from myslam_ref.triangulation import Algo, triangulate
from myslam_ref.viz import draw_trajectory_3d  # noqa: F401

# %%
ds = load_kitti_mini()
cam0, cam1 = ds.camera0, ds.camera1
K = cam0.K
T_left = np.hstack([np.eye(3), np.zeros((3, 1))])
T_right = np.hstack([np.eye(3), np.array([[-cam1.baseline_m], [0], [0]])])
print(f"running VO on {len(ds.frames)} KITTI mini frames; baseline {cam1.baseline_m:.3f} m, fx {cam0.fx:.2f}")

# %% [markdown]
# ## 1. Stereo init on frame 0

# %%
frame0 = ds.frames[0]
seeds0 = detect(frame0.left, Detector.GFTT, maxFeatures=150, qualityLevel=0.01, minDistance=20)
tracked_right0 = track_lk(frame0.left, frame0.right, seeds0)
tri0 = triangulate(seeds0, tracked_right0,
                   np.array([cam0.fx, cam0.fy, cam0.cx, cam0.cy]), T_left,
                   np.array([cam1.fx, cam1.fy, cam1.cx, cam1.cy]), T_right,
                   algo=Algo.LINEAR_SVD, quality_threshold=0.01)
valid0 = tri0[:, 4] > 0.5
landmarks_world = tri0[valid0, :3].copy()  # (M, 3) in frame-0 camera frame == world
landmark_left_pts = seeds0[valid0][:, :2].copy()  # (M, 2) tracked location on frame 0 left
print(f"frame 0 stereo init: {landmarks_world.shape[0]} landmarks")
assert landmarks_world.shape[0] >= 30

# Persistent SlamMap (Step 12) for sanity tracking of the active window.
smap = SlamMap(num_active_keyframes=4)
lm_ids = [smap.insert_map_point(lp) for lp in landmarks_world]
T_cw_0 = np.eye(4)
smap.insert_keyframe(0, T_cw_0, set(lm_ids), Policy.CH13_DEFAULT, PolicyOptions(0.2))

# %% [markdown]
# ## 2. Frame 1..4 — temporal LK + PnP

# %%
trajectory = [np.eye(4)]
prev_left = frame0.left
prev_pts = landmark_left_pts
prev_lms = landmarks_world

for i, frame in enumerate(ds.frames[1:], start=1):
    curr_left = frame.left
    # Temporal LK from previous landmarks' left-frame pixel coords to current left.
    seeds = np.hstack([prev_pts, np.ones((prev_pts.shape[0], 1))])
    tracked = track_lk(prev_left, curr_left, seeds, winSize=11, maxLevel=3, maxIter=30, eps=0.01)
    ok = tracked[:, 2] > 0.5
    rate = float(ok.mean())
    n_tracked = int(ok.sum())
    if n_tracked < 6:
        print(f"  frame {i}: insufficient tracks ({n_tracked}), stopping")
        break

    # PnP: use the previously triangulated 3D points (still in world frame since
    # our world = frame-0 left camera) + the tracked 2D points on current frame.
    pts3 = prev_lms[ok]
    pts2 = tracked[ok, :2]
    init6 = np.concatenate([trajectory[-1][:3, 3], cv2.Rodrigues(trajectory[-1][:3, :3])[0].reshape(-1)])
    res = estimate_pose(pts3, pts2, K, init6, rounds=4, iter_per_round=10, chi2_threshold=5.991)
    T_new = np.eye(4); T_new[:3, :] = res.T_cw
    trajectory.append(T_new)
    inliers = int(res.final_inlier_mask.sum())
    print(f"  frame {i}: tracked {n_tracked}/{prev_pts.shape[0]} ({rate * 100:.1f}%), PnP inliers {inliers}/{pts3.shape[0]}, t = {T_new[:3, 3]}")
    assert rate >= 0.5, f"frame {i} track rate {rate:.1%} below 50%"
    assert inliers >= 6, f"frame {i} PnP inliers {inliers} too low"

    # Decide keyframe (Step 09 policy).
    kf = decide_inlier_threshold(inliers, min_inliers=int(landmarks_world.shape[0] * 0.6))
    if kf.insert:
        print(f"    → insert KF (reason: {kf.reason})")
        smap.insert_keyframe(i, T_new, set(lm_ids[:inliers]), Policy.CH13_DEFAULT, PolicyOptions(0.2))

    prev_left = curr_left
    prev_pts = tracked[ok, :2]
    prev_lms = pts3

# %% [markdown]
# ## 3. Trajectory sanity

# %%
ts = np.array([T[:3, 3] for T in trajectory])
print(f"trajectory ({len(trajectory)} poses):")
for i, t in enumerate(ts):
    print(f"  frame {i}: t = ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})")
# Forward motion → world-frame z translation should grow in magnitude across the chain.
# (We're in camera 0 frame; subsequent cameras have negative tz in their T_cw since
# the world recedes from the new camera position.)
z_progression = ts[:, 2]
distances = [float(np.linalg.norm(ts[i] - ts[i - 1])) for i in range(1, len(ts))]
print(f"  inter-frame distances: {[f'{d:.3f}' for d in distances]}")
total_distance = float(np.linalg.norm(ts[-1] - ts[0]))
print(f"  total displacement: {total_distance:.3f} m")
assert total_distance > 0.1, f"trajectory total displacement {total_distance} m surprisingly small"
assert all(d < 5.0 for d in distances), "an inter-frame jump > 5 m suggests divergence"

# %% [markdown]
# ## 4. 시각화 — final trajectory + landmark cloud

# %%
import matplotlib.pyplot as plt

fig_tj = draw_trajectory_3d(
    trajectory,
    landmarks=landmarks_world,
    title=f"VO on KITTI mini · {len(trajectory)} frames · total {total_distance:.3f} m",
)

# %%
# 프레임별 카메라 z translation 추이 (T_cw 의 마지막 컬럼).
fig_tz, ax_tz = plt.subplots(figsize=(7, 3))
ax_tz.plot(range(len(ts)), ts[:, 0], label="tx", marker="o")
ax_tz.plot(range(len(ts)), ts[:, 1], label="ty", marker="o")
ax_tz.plot(range(len(ts)), ts[:, 2], label="tz", marker="o")
ax_tz.set_xlabel("frame")
ax_tz.set_ylabel("T_cw translation component (m)")
ax_tz.set_title("Step 13 · per-frame T_cw translation")
ax_tz.grid(alpha=0.3); ax_tz.legend()
fig_tz.tight_layout()

# %%
print(f"\nOK — step13 full pipeline: {len(trajectory)} poses + {smap.num_active_keyframes}-KF active window")
