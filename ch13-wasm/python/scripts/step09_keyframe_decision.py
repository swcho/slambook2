# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 09 — Keyframe decision
#
# 세 가지 정책을 한 합성 시퀀스에 모두 적용하고 각 정책이 다른 시점에 발화한다는
# 점을 확인한다.

# %%
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np

from myslam_ref.keyframe import decide_fixed_interval, decide_frame_distance, decide_inlier_threshold
from myslam_ref.se3 import se3_from_translation
from myslam_ref.viz import draw_keyframe_timeline  # noqa: F401

# %% [markdown]
# ## 합성 시퀀스
#
# 10 프레임, 평균 inlier 수는 점차 줄어들고, 카메라는 매 프레임 +z 로 0.3 m 이동.

# %%
inlier_counts = [150, 140, 130, 125, 120, 115, 95, 88, 78, 60]  # 마지막 4 프레임만 < 80
poses = [se3_from_translation([0, 0, i * 0.3]) for i in range(10)]
frames = list(range(10))

# %% [markdown]
# ## 정책 1: inlier threshold (< 80)

# %%
inserts_p1 = [decide_inlier_threshold(c, min_inliers=80).insert for c in inlier_counts]
print("inlier_threshold inserts at frames:", [i for i, b in enumerate(inserts_p1) if b])
assert inserts_p1 == [False] * 8 + [True] * 2

# %% [markdown]
# ## 정책 2: frame distance (≥ 0.5 m)
#
# 0.3 m / step → 2 step (= 0.6 m) 마다 발화.

# %%
last_kf_pose = poses[0]
inserts_p2 = []
for i, pose in enumerate(poses):
    if i == 0:
        inserts_p2.append(False)
        continue
    d = decide_frame_distance(pose, last_kf_pose, min_distance_m=0.5)
    inserts_p2.append(d.insert)
    if d.insert:
        last_kf_pose = pose
print("frame_distance inserts at frames:", [i for i, b in enumerate(inserts_p2) if b])
assert inserts_p2.count(True) >= 4  # ~ 매 2 step 마다 발화

# %% [markdown]
# ## 정책 3: fixed interval (≥ 5 frames)

# %%
last_kf_frame = 0
inserts_p3 = []
for i in frames:
    if i == 0:
        inserts_p3.append(False)
        continue
    d = decide_fixed_interval(i, last_kf_frame, interval=5)
    inserts_p3.append(d.insert)
    if d.insert:
        last_kf_frame = i
print("fixed_interval inserts at frames:", [i for i, b in enumerate(inserts_p3) if b])
assert inserts_p3 == [False] * 5 + [True] + [False] * 4

# %% [markdown]
# ## 4. 시각화 — 정책별 KF 삽입 타임라인

# %%
fig1 = draw_keyframe_timeline(frames, inserts_p1, metric=inlier_counts, title="policy 1 · inlier threshold (< 80)")
fig2 = draw_keyframe_timeline(frames, inserts_p2, title="policy 2 · frame distance (≥ 0.5 m)")
fig3 = draw_keyframe_timeline(frames, inserts_p3, title="policy 3 · fixed interval (≥ 5 frames)")

# %%
print("OK — step09 keyframe decision policies fire on the expected frames")
