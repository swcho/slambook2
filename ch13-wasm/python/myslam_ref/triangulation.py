"""Stereo triangulation — Python mirror of bind_triangulation.cpp.

Two algorithms are supported, matching the C++ binding's enum:

- ``Algo.LINEAR_SVD``: DLT — stack the four projection equations into a 4×4 A
  and solve via SVD. ``quality = σ4 / σ3`` (small = well-conditioned). Mirrors
  ch13/src/triangulation.cpp and the spike binding.
- ``Algo.MIDPOINT``: closed-form midpoint between the two rays, again with a
  σ4/σ3 quality. ``Algo.MIDPOINT`` here returns 0.0 for quality (matches the
  C++ binding's degenerate handling — it has no equivalent metric).

Output layout matches the C++ side: an (N, 5) Float64 array with columns
``[x, y, z, ratio, ok]`` so per-row inspection is one slice away.
"""

from __future__ import annotations

from enum import IntEnum

import numpy as np


class Algo(IntEnum):
    LINEAR_SVD = 0
    MIDPOINT = 1


def _camera_K_matrix(k4: np.ndarray) -> np.ndarray:
    """``k4 = [fx, fy, cx, cy]`` → 3×3 intrinsic."""
    return np.array([[k4[0], 0.0, k4[2]], [0.0, k4[1], k4[3]], [0.0, 0.0, 1.0]], dtype=np.float64)


def _projection_matrix(K: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Build the 3×4 projection ``K · [R | t]`` for a row-major 3×4 ``T``."""
    T = T.reshape(3, 4) if T.size == 12 else T[:3, :4]
    return K @ T


def _dlt_one(pl: np.ndarray, pr: np.ndarray, Pl: np.ndarray, Pr: np.ndarray) -> tuple[np.ndarray, float]:
    """Solve a single point via DLT, return (Xh_normalized, σ4/σ3)."""
    ul, vl = pl
    ur, vr = pr
    A = np.array([
        vl * Pl[2] - Pl[1],
        Pl[0] - ul * Pl[2],
        vr * Pr[2] - Pr[1],
        Pr[0] - ur * Pr[2],
    ])
    _u, s, vh = np.linalg.svd(A)
    Xh = vh[-1]
    if Xh[3] != 0.0:
        Xh = Xh / Xh[3]
    # σ4 / σ3 — small ratio = well-conditioned.
    s3 = s[2] if s.size >= 3 else np.inf
    ratio = float(s[-1] / s3) if s3 > 0 else float("inf")
    return Xh[:3], ratio


def _midpoint_one(pl: np.ndarray, pr: np.ndarray, K_l: np.ndarray, T_l: np.ndarray, K_r: np.ndarray, T_r: np.ndarray) -> np.ndarray:
    """Midpoint between two rays in world frame.

    Rays: x = c + α·d. Cameras at world-frame centres ``c_l, c_r``; ray
    directions ``d_l, d_r`` are the back-projected pixels normalised.
    """
    # World-frame centres: c_world = -R^T · t for T = [R|t].
    R_l, t_l = T_l[:, :3], T_l[:, 3]
    R_r, t_r = T_r[:, :3], T_r[:, 3]
    c_l = -R_l.T @ t_l
    c_r = -R_r.T @ t_r
    # Back-project pixel through camera → camera-frame ray → world-frame ray.
    ray_l_cam = np.linalg.solve(K_l, np.array([pl[0], pl[1], 1.0]))
    ray_r_cam = np.linalg.solve(K_r, np.array([pr[0], pr[1], 1.0]))
    d_l = R_l.T @ ray_l_cam
    d_r = R_r.T @ ray_r_cam
    d_l /= np.linalg.norm(d_l)
    d_r /= np.linalg.norm(d_r)
    # Solve for α, β minimising ‖(c_l + α d_l) − (c_r + β d_r)‖².
    # Normal equations: [d_l·d_l, −d_l·d_r; −d_l·d_r, d_r·d_r] [α; β] = [d_l·(c_r−c_l); −d_r·(c_r−c_l)]
    a = d_l @ d_l
    b = -d_l @ d_r
    c = d_r @ d_r
    rhs0 = d_l @ (c_r - c_l)
    rhs1 = -d_r @ (c_r - c_l)
    det = a * c - b * b
    if abs(det) < 1e-12:
        return np.array([0.0, 0.0, 0.0])
    alpha = (c * rhs0 - b * rhs1) / det
    beta = (a * rhs1 - b * rhs0) / det
    p_l = c_l + alpha * d_l
    p_r = c_r + beta * d_r
    return 0.5 * (p_l + p_r)


def triangulate(
    left_pts: np.ndarray,
    right_pts: np.ndarray,
    k_left: np.ndarray,
    t_left: np.ndarray,
    k_right: np.ndarray,
    t_right: np.ndarray,
    *,
    algo: Algo | int = Algo.LINEAR_SVD,
    quality_threshold: float = 0.01,
    inverted_return: bool = False,
) -> np.ndarray:
    """Stereo triangulation. Matches bind_triangulation.cpp output layout (N, 5).

    Inputs:
      - ``left_pts``, ``right_pts``: (N, 3) arrays with ``[x, y, score|status]``.
        ``right_pts[:, 2] <= 0.5`` rows are treated as "lost" → row zeroed
        with ``ratio = NaN``.
      - ``k_left``, ``k_right``: either ``(4,)`` ``[fx, fy, cx, cy]`` or ``(3, 3)``.
      - ``t_left``, ``t_right``: ``(3, 4)`` (or length-12 row-major) projection extrinsics ``[R | t]``.

    Output columns: ``[x, y, z, ratio, ok]``.
    """
    L = np.asarray(left_pts, dtype=np.float64).reshape(-1, 3)
    R = np.asarray(right_pts, dtype=np.float64).reshape(-1, 3)
    n = L.shape[0]
    if R.shape[0] != n:
        raise ValueError(f"left/right size mismatch: {n} vs {R.shape[0]}")

    K_l = np.asarray(k_left, dtype=np.float64)
    K_r = np.asarray(k_right, dtype=np.float64)
    if K_l.size == 4:
        K_l = _camera_K_matrix(K_l)
    if K_r.size == 4:
        K_r = _camera_K_matrix(K_r)
    T_l = np.asarray(t_left, dtype=np.float64).reshape(3, 4)
    T_r = np.asarray(t_right, dtype=np.float64).reshape(3, 4)
    P_l = _projection_matrix(K_l, T_l)
    P_r = _projection_matrix(K_r, T_r)

    out = np.zeros((n, 5), dtype=np.float64)
    for i in range(n):
        if R[i, 2] <= 0.5:
            out[i] = [0.0, 0.0, 0.0, np.nan, 0.0]
            continue
        if int(algo) == int(Algo.LINEAR_SVD):
            xyz, ratio = _dlt_one(L[i, :2], R[i, :2], P_l, P_r)
        elif int(algo) == int(Algo.MIDPOINT):
            xyz = _midpoint_one(L[i, :2], R[i, :2], K_l, T_l, K_r, T_r)
            ratio = 0.0
        else:
            raise ValueError(f"unknown algo: {algo}")
        # depth check (must be in front of both cameras for a basic gate; we
        # mirror the C++ binding which just checks left depth z > 0).
        # Compute depth in left camera: (R_l · X + t_l)[2].
        pc_l = T_l[:, :3] @ xyz + T_l[:, 3]
        depth_ok = pc_l[2] > 0.0
        quality_ok = ratio < quality_threshold
        ok = depth_ok and (quality_ok != inverted_return)
        out[i] = [xyz[0], xyz[1], xyz[2], ratio, 1.0 if ok else 0.0]
    return out
