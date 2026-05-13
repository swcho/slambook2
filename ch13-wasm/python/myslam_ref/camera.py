"""Pinhole camera math — Python mirror of ``wasm-src/myslam/bindings/bind_camera.cpp``.

The WASM ``Camera`` class assumes T_c_w = identity and treats the stereo
extrinsic as a pure translation ``(tx, ty, tz)`` (left rig identity, right
shifted by ``-baseline`` along x for rectified KITTI). We mirror that here and
add a more general ``T_cw`` variant for later steps that need SE(3) poses.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Camera:
    """Mirrors the embind ``Camera`` class. T_c_w fixed at identity."""

    fx: float
    fy: float
    cx: float
    cy: float
    baseline: float
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]], dtype=np.float64)

    # ---- Single-point operations (match the embind method names) -------------

    def camera_to_pixel(self, xyz_c: np.ndarray | tuple[float, float, float]) -> np.ndarray:
        xc, yc, zc = float(xyz_c[0]), float(xyz_c[1]), float(xyz_c[2])
        return np.array([self.fx * xc / zc + self.cx, self.fy * yc / zc + self.cy], dtype=np.float64)

    def pixel_to_camera(self, uv: np.ndarray | tuple[float, float], depth: float) -> np.ndarray:
        u, v = float(uv[0]), float(uv[1])
        return np.array(
            [(u - self.cx) * depth / self.fx, (v - self.cy) * depth / self.fy, depth], dtype=np.float64
        )

    def world_to_pixel(self, xyz_w: np.ndarray | tuple[float, float, float]) -> np.ndarray:
        # T_c_w = identity; rig-to-camera extrinsic is a pure translation.
        xc = xyz_w[0] + self.tx
        yc = xyz_w[1] + self.ty
        zc = xyz_w[2] + self.tz
        return self.camera_to_pixel((xc, yc, zc))

    def pixel_to_world(self, uv: np.ndarray | tuple[float, float], depth: float) -> np.ndarray:
        xyz_c = self.pixel_to_camera(uv, depth)
        return np.array([xyz_c[0] - self.tx, xyz_c[1] - self.ty, xyz_c[2] - self.tz], dtype=np.float64)


# ---- Batch operations (match the embind free functions) ---------------------


def project_batch(cam: Camera, xyz_c: np.ndarray) -> np.ndarray:
    """Project an (N, 3) array of camera-frame points to (N, 2) pixels.

    Mirrors ``projectBatch`` in bind_camera.cpp — no T_c_w, no clipping.
    """
    xyz = np.asarray(xyz_c, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f"project_batch expects (N, 3); got {xyz.shape}")
    z = xyz[:, 2]
    u = cam.fx * xyz[:, 0] / z + cam.cx
    v = cam.fy * xyz[:, 1] / z + cam.cy
    return np.stack([u, v], axis=1)


def round_trip_max_error(cam: Camera, xyz_c: np.ndarray) -> float:
    """For each (xc, yc, zc) with zc > 0: project to (u, v) then unproject at zc,
    and report ``max ||(xc, yc) - (xc', yc')||``. Mirrors ``roundTripMaxError``.
    """
    xyz = np.asarray(xyz_c, dtype=np.float64)
    if xyz.shape[0] == 0:
        return 0.0
    z = xyz[:, 2]
    valid = z > 0.0
    if not np.any(valid):
        return 0.0
    xv = xyz[valid]
    u = cam.fx * xv[:, 0] / xv[:, 2] + cam.cx
    v = cam.fy * xv[:, 1] / xv[:, 2] + cam.cy
    x_back = (u - cam.cx) * xv[:, 2] / cam.fx
    y_back = (v - cam.cy) * xv[:, 2] / cam.fy
    dx = xv[:, 0] - x_back
    dy = xv[:, 1] - y_back
    return float(np.max(np.hypot(dx, dy)))


# ---- Generic SE(3) helpers (for later steps) --------------------------------


def project_with_pose(K: np.ndarray, T_cw: np.ndarray, points_w: np.ndarray) -> np.ndarray:
    """Project world-frame points through a generic SE(3) pose.

    K: (3, 3); T_cw: (4, 4); points_w: (N, 3) → (N, 2).
    """
    K = np.asarray(K, dtype=np.float64)
    T = np.asarray(T_cw, dtype=np.float64)
    pts = np.asarray(points_w, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    pc = pts @ R.T + t  # (N, 3) in camera frame
    uv_h = pc @ K.T  # (N, 3)
    return uv_h[:, :2] / uv_h[:, 2:3]
