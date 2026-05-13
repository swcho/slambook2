# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 10 — New map points (mask-existing + redetect + triangulate)
#
# ch13 `Frontend::TriangulateNewPoints` 흐름:
#
# 1. 새 left frame 에서 기존 트랙된 점 주변 ±10 px 마스크
# 2. 마스크 위치 빼고 재검출 (`bind_features.cpp` 의 mask 옵션 활용)
# 3. 새 점들을 stereo LK → triangulate
# 4. 깊이 합리적인 점만 채택

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

# %%
ds = load_kitti_mini()
cam = ds.camera0

# Frame 0 의 stereo init 으로 "기존 점"을 만든다.
left0, right0 = ds.frames[0].left, ds.frames[0].right
seeds0 = detect(left0, Detector.GFTT, maxFeatures=120, qualityLevel=0.01, minDistance=20)
tracked0 = track_lk(left0, right0, seeds0)
ok0 = tracked0[:, 2] > 0.5
print(f"frame 0 stereo: {int(ok0.sum())}/{seeds0.shape[0]} tracked")
# 기존 점 좌표 (frame 0 시점에 추적되어 있다고 가정).
existing_pts = seeds0[ok0]

# %% [markdown]
# ## 1. 마스크 생성 — 기존 점 주변 20×20 픽셀 제외

# %%
h, w = left0.shape
mask = np.full((h, w), 255, dtype=np.uint8)
PAD = 10
for x, y, _ in existing_pts:
    xi, yi = int(x), int(y)
    x0, x1 = max(0, xi - PAD), min(w, xi + PAD + 1)
    y0, y1 = max(0, yi - PAD), min(h, yi + PAD + 1)
    mask[y0:y1, x0:x1] = 0
masked_ratio = float((mask == 0).sum()) / mask.size
print(f"masked area ratio: {masked_ratio * 100:.1f}%")

# %% [markdown]
# ## 2. 마스크 적용 재검출

# %%
new_seeds = detect(left0, Detector.GFTT, maxFeatures=200, qualityLevel=0.01, minDistance=20, mask=mask)
print(f"new seeds (mask excluded): {new_seeds.shape[0]}")
assert new_seeds.shape[0] > 0

# 검출된 새 점은 모두 기존 점에서 ≥ ``PAD`` 픽셀 떨어져 있어야 한다.
def min_distance(pt, others):
    d = np.linalg.norm(others[:, :2] - pt[:2], axis=1)
    return float(d.min()) if d.size else float("inf")

for new_pt in new_seeds:
    d = min_distance(new_pt, existing_pts)
    assert d >= PAD - 1, f"new pt {new_pt} too close (d={d}) to existing"

# %% [markdown]
# ## 3. Stereo LK + triangulate the new seeds

# %%
new_tracked = track_lk(left0, right0, new_seeds)
k_arr = np.array([cam.fx, cam.fy, cam.cx, cam.cy])
T_l = np.hstack([np.eye(3), np.zeros((3, 1))])
T_r = np.hstack([np.eye(3), np.array([[-ds.camera1.baseline_m], [0], [0]])])
new_pts3 = triangulate(new_seeds, new_tracked, k_arr, T_l, k_arr, T_r)
new_accepted = new_pts3[new_pts3[:, 4] > 0.5]
print(f"new landmarks accepted: {new_accepted.shape[0]}/{new_seeds.shape[0]}")
assert new_accepted.shape[0] >= 10

# %%
print(f"OK — step10 new map points: produced {new_accepted.shape[0]} fresh landmarks under mask exclusion")
