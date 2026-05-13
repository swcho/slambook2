# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 02 — Pinhole camera 모델
#
# `wasm-src/myslam/bindings/bind_camera.cpp` 의 ``Camera`` 클래스와 동일한 의미의
# Python 레퍼런스. T_c_w = identity, 스테레오 외부 파라미터는 순수 translation
# (rectified KITTI 가정).

# %%
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np

from myslam_ref.camera import Camera, project_batch, project_with_pose, round_trip_max_error
from myslam_ref.dataset import load_kitti_mini

# %% [markdown]
# ## 1. KITTI mini calib 로 Camera 객체 만들기

# %%
ds = load_kitti_mini()
cam0 = Camera(
    fx=ds.camera0.fx, fy=ds.camera0.fy, cx=ds.camera0.cx, cy=ds.camera0.cy,
    baseline=0.0, tx=0.0, ty=0.0, tz=0.0,
)
cam1 = Camera(
    fx=ds.camera1.fx, fy=ds.camera1.fy, cx=ds.camera1.cx, cy=ds.camera1.cy,
    baseline=ds.camera1.baseline_m,
    tx=ds.camera1.t[0], ty=ds.camera1.t[1], tz=ds.camera1.t[2],
)
print(f"camera0: fx={cam0.fx:.4f} t=({cam0.tx:.4f},{cam0.ty:.4f},{cam0.tz:.4f}) baseline={cam0.baseline:.6f}")
print(f"camera1: fx={cam1.fx:.4f} t=({cam1.tx:+.4f},{cam1.ty:+.4f},{cam1.tz:+.4f}) baseline={cam1.baseline:.6f}")

# %% [markdown]
# ## 2. 단일 점 round-trip
#
# 카메라 좌표계 점을 픽셀로 투영한 뒤 같은 depth 로 역투영하면 원본과 일치해야 한다.

# %%
xyz_in = (0.5, -0.3, 7.2)
uv = cam0.camera_to_pixel(xyz_in)
xyz_back = cam0.pixel_to_camera(uv, depth=xyz_in[2])
err = float(np.linalg.norm(np.asarray(xyz_in) - xyz_back))
print(f"single round-trip: input={xyz_in}, uv={uv}, back={xyz_back}, |err|={err:.2e}")
assert err < 1e-10

# %% [markdown]
# ## 3. 배치 round-trip (1000 random points)

# %%
rng = np.random.default_rng(seed=42)
pts_c = rng.normal(scale=2.0, size=(1000, 3))
pts_c[:, 2] = np.abs(pts_c[:, 2]) + 1.0  # ensure depth > 0
err_max = round_trip_max_error(cam0, pts_c)
print(f"batch round-trip max error: {err_max:.2e} (n=1000)")
assert err_max < 1e-10

# %% [markdown]
# ## 4. project_batch shape sanity

# %%
uv_batch = project_batch(cam0, pts_c)
assert uv_batch.shape == (1000, 2)
# Spot-check: first row matches single-point projection
expected_uv0 = cam0.camera_to_pixel(pts_c[0])
assert np.allclose(uv_batch[0], expected_uv0)

# %% [markdown]
# ## 5. Stereo baseline 검증
#
# rectified pair 에서 left 와 right 의 픽셀 차이는 ``disparity = fx * baseline / depth``.
# world point ``(0, 0, Z)`` 를 두 카메라에 투영한 뒤 ``u_left - u_right`` 와 비교.

# %%
Z = 10.0
world_pt = (0.0, 0.0, Z)
uv_left = cam0.world_to_pixel(world_pt)
uv_right = cam1.world_to_pixel(world_pt)
disparity_obs = uv_left[0] - uv_right[0]
disparity_pred = cam0.fx * cam1.baseline / Z
print(f"depth={Z} m → observed disparity={disparity_obs:.4f} px, predicted={disparity_pred:.4f} px")
assert abs(disparity_obs - disparity_pred) < 1e-6

# %% [markdown]
# ## 6. SE(3) project_with_pose smoke
#
# T_cw = identity 이면 ``project_with_pose`` 가 ``project_batch`` 와 동일해야 한다.

# %%
T_id = np.eye(4)
uv_pose = project_with_pose(cam0.K, T_id, pts_c)
assert np.allclose(uv_pose, uv_batch, atol=1e-10)

# Non-identity pose: rotate 90° around y and translate 2 m along z.
T = np.eye(4)
theta = np.pi / 2
T[:3, :3] = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])
T[:3, 3] = [0, 0, 2]
uv_rot = project_with_pose(cam0.K, T, np.array([[1.0, 0.0, 5.0]]))
# Manual check: R @ (1, 0, 5) = (cos*1 + sin*5, 0, -sin*1 + cos*5) = (5, 0, -1) at θ=π/2.
# Add t = (0, 0, 2) → pc = (5, 0, 1). Project: u = fx * 5 / 1 + cx, v = fy * 0 / 1 + cy = cy.
pc_pred = np.array([5.0, 0.0, 1.0])
expected_u = cam0.fx * pc_pred[0] / pc_pred[2] + cam0.cx
expected_v = cam0.fy * pc_pred[1] / pc_pred[2] + cam0.cy
assert abs(uv_rot[0, 0] - expected_u) < 1e-6, f"expected u={expected_u}, got {uv_rot[0, 0]}"
assert abs(uv_rot[0, 1] - expected_v) < 1e-6, f"expected v={expected_v}, got {uv_rot[0, 1]}"

# %%
print("OK — step02 camera reference passes pinhole + baseline + SE(3) checks")
