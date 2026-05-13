# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 07 — Frame tracking (temporal LK)
#
# ch13 `Frontend::TrackLastFrame` 과 같은 흐름: 이전 left frame 에서 GFTT 시드를
# 잡고 현재 left frame 으로 LK 트래킹. KITTI mini 의 5 개 프레임에 대해
# pairwise 트래킹을 확인한다.

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

# %% [markdown]
# ## 1. Pairwise tracking — 5 frame chain

# %%
results = []
prev = ds.frames[0].left
seeds_prev = detect(prev, Detector.GFTT, maxFeatures=200, qualityLevel=0.01, minDistance=20)
for i, frame in enumerate(ds.frames[1:], start=1):
    curr = frame.left
    tracked = track_lk(prev, curr, seeds_prev, winSize=11, maxLevel=3, maxIter=30, eps=0.01)
    ok = tracked[:, 2] > 0.5
    n = int(ok.sum())
    rate = n / seeds_prev.shape[0]
    dx = float((tracked[ok, 0] - seeds_prev[ok, 0]).mean()) if n else float("nan")
    dy = float((tracked[ok, 1] - seeds_prev[ok, 1]).mean()) if n else float("nan")
    print(f"frame {i - 1} → {i}: tracked {n}/{seeds_prev.shape[0]} ({rate * 100:.1f}%), mean (dx,dy)=({dx:+.2f},{dy:+.2f})")
    results.append((rate, dx, dy))
    # Re-seed from current frame for next pair (ch13 frontend pattern).
    prev = curr
    seeds_prev = detect(prev, Detector.GFTT, maxFeatures=200, qualityLevel=0.01, minDistance=20)

# %% [markdown]
# ## 2. Gate
#
# - 모든 페어가 ≥ 70 % 트래킹
# - KITTI 05 의 forward motion 은 ``dx`` 가 작고 (직진), ``dy`` 도 작음 (도로 평면)

# %%
for i, (rate, dx, dy) in enumerate(results):
    assert rate >= 0.7, f"pair {i}→{i + 1}: rate {rate:.1%} < 70%"
    # Forward motion 이지만 KITTI mini 1226×370 에서 픽셀 단위 dx 는 보통 -50 ~ +50 px.
    assert abs(dx) < 100, f"pair {i}→{i + 1}: |dx|={abs(dx):.1f} surprisingly large"
    assert abs(dy) < 30, f"pair {i}→{i + 1}: |dy|={abs(dy):.1f} surprisingly large"

# %% [markdown]
# ## 3. 시각화 — frame 0→1 temporal flow

# %%
prev0 = ds.frames[0].left
curr1 = ds.frames[1].left
seeds0 = detect(prev0, Detector.GFTT, maxFeatures=200, qualityLevel=0.01, minDistance=20)
tracked01 = track_lk(prev0, curr1, seeds0)
fig_flow = draw_flow_field(
    prev0, seeds0[:, :2], tracked01[:, :2], status=tracked01[:, 2],
    title=f"KITTI mini · temporal LK frame 0 → 1 (rate {(tracked01[:, 2] > 0.5).mean() * 100:.0f}%)",
    step=2,
)

# %%
print(f"OK — step07 temporal LK tracks {len(results)} consecutive pairs with rate ≥ 70 %")
