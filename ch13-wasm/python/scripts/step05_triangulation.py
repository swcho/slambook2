# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 05 — Stereo triangulation
#
# `wasm-src/spike/verify_triangulation.ts` 의 4 가지 케이스를 모두 미러:
#
# 1. Exact correspondences → LinearSVD/Midpoint maxErr < 1e-6 m
# 2. Epipolar noise → σ4/σ3 비율 상승
# 3. ``inverted_return`` 토글 → ok flag 반전
# 4. ``status ≤ 0.5`` 입력 → ``(0, 0, 0, NaN, 0)``

# %%
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np

from myslam_ref.triangulation import Algo, triangulate
from myslam_ref.viz import draw_pointcloud_3d, draw_quality_histogram  # noqa: F401

# %%
# KITTI 05 @ 0.5× downsample (verify_triangulation.ts 와 동일).
fx, fy, cx, cy = 360.295, 360.295, 303.605, 92.695
baseline = 0.537151
k = np.array([fx, fy, cx, cy])
T_l = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]], dtype=np.float64)
T_r = np.array([[1, 0, 0, -baseline], [0, 1, 0, 0], [0, 0, 1, 0]], dtype=np.float64)

# %% [markdown]
# ## Case A — 정확한 대응점, GT recovery

# %%
gt = []
for z_step in range(4):
    z = 5 + z_step * 4
    for x_step in range(-2, 3):
        for y_step in range(-1, 2):
            gt.append([x_step * 1.5, y_step * 0.8, z])
gt = np.asarray(gt, dtype=np.float64)
N = gt.shape[0]

# Project to left/right.
left_pts = np.zeros((N, 3))
right_pts = np.zeros((N, 3))
for i, (x, y, z) in enumerate(gt):
    left_pts[i] = [fx * x / z + cx, fy * y / z + cy, 1.0]
    xr = x - baseline
    right_pts[i] = [fx * xr / z + cx, fy * y / z + cy, 1.0]

svd = triangulate(left_pts, right_pts, k, T_l, k, T_r, algo=Algo.LINEAR_SVD, quality_threshold=0.01)
err_svd = np.linalg.norm(svd[:, :3] - gt, axis=1)
print(f"LinearSVD: maxErr={err_svd.max():.2e} m, maxRatio={svd[:, 3].max():.2e}, accepted={int((svd[:, 4] > 0.5).sum())}/{N}")
assert err_svd.max() < 1e-6
assert svd[:, 3].max() < 1e-9
assert int((svd[:, 4] > 0.5).sum()) == N

mid = triangulate(left_pts, right_pts, k, T_l, k, T_r, algo=Algo.MIDPOINT, quality_threshold=0.01)
err_mid = np.linalg.norm(mid[:, :3] - gt, axis=1)
print(f"Midpoint:  maxErr={err_mid.max():.2e} m, accepted={int((mid[:, 4] > 0.5).sum())}/{N}")
assert err_mid.max() < 1e-6
assert int((mid[:, 4] > 0.5).sum()) == N

# %% [markdown]
# ## Case B — 4 px epipolar noise → σ4/σ3 상승

# %%
noisy_right = right_pts.copy()
noisy_right[:, 1] += 4.0
noisy = triangulate(left_pts, noisy_right, k, T_l, k, T_r, algo=Algo.LINEAR_SVD, quality_threshold=0.01)
median_ratio = float(np.median(noisy[:, 3]))
print(f"4 px epipolar noise: medianRatio={median_ratio:.2e}, accepted={int((noisy[:, 4] > 0.5).sum())}/{N}")
assert median_ratio > 1e-6

# %% [markdown]
# ## Case C — ``inverted_return`` 토글

# %%
fixed = triangulate(left_pts, noisy_right, k, T_l, k, T_r, algo=Algo.LINEAR_SVD, quality_threshold=1e-3, inverted_return=False)
inverted = triangulate(left_pts, noisy_right, k, T_l, k, T_r, algo=Algo.LINEAR_SVD, quality_threshold=1e-3, inverted_return=True)
fixed_ok = (fixed[:, 4] > 0.5)
inv_ok = (inverted[:, 4] > 0.5)
overlap = int(np.sum(fixed_ok == inv_ok))
print(f"inverted toggle: fixed={int(fixed_ok.sum())}/{N} inverted={int(inv_ok.sum())}/{N} overlap={overlap}")
assert overlap == 0, f"inverted should flip every row, got {overlap} matching"
assert int(fixed_ok.sum()) + int(inv_ok.sum()) == N

# %% [markdown]
# ## Case D — ``status ≤ 0.5`` 입력 → 행 zero-out

# %%
lost_right = right_pts.copy()
lost_right[0, 2] = 0.0
lost_right[5, 2] = 0.0
with_lost = triangulate(left_pts, lost_right, k, T_l, k, T_r, algo=Algo.LINEAR_SVD)
for idx in [0, 5]:
    row = with_lost[idx]
    assert row[0] == 0 and row[1] == 0 and row[2] == 0
    assert np.isnan(row[3])
    assert row[4] == 0
# Non-lost rows continue triangulating normally.
for idx in [1, 2, 3, 4]:
    assert with_lost[idx, 4] == 1.0

# %% [markdown]
# ## 5. 시각화

# %%
fig_pc = draw_pointcloud_3d(
    svd[:, :3],
    extra_clouds={"ground truth": gt},
    title="LinearSVD recovered (estimate vs GT)",
)

# %%
fig_q = draw_quality_histogram(
    noisy[:, 3],
    threshold=0.01,
    title="4 px epipolar noise · σ4/σ3 distribution",
)

# %%
print("OK — step05 triangulation passes 4-case gate (LinearSVD + Midpoint + invert + lost rows)")
