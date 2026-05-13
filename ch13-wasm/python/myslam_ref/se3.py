"""Minimal SE(3) helpers — Python mirror of src/lib/slam/se3.ts.

Used by sliding-window map management (Step 12) and the keyframe distance
policy (Step 09). Matches the TS implementation's left-Jacobian convention so
``se3_log_norm`` returns the same value for the same pose.

All matrices are float64 4×4 (row-major in memory but accessed as 2-D arrays).
"""

from __future__ import annotations

import numpy as np


def mat4_identity() -> np.ndarray:
    return np.eye(4, dtype=np.float64)


def se3_invert(m: np.ndarray) -> np.ndarray:
    """``[R | t]`` → ``[Rᵀ | -Rᵀ t]``."""
    m = np.asarray(m, dtype=np.float64).reshape(4, 4)
    R = m[:3, :3]
    t = m[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def se3_translation(m: np.ndarray) -> np.ndarray:
    return np.asarray(m, dtype=np.float64).reshape(4, 4)[:3, 3].copy()


def se3_log_norm(m: np.ndarray) -> float:
    """Compute ‖log(T)‖ for ``T ∈ SE(3)`` — matches src/lib/slam/se3.ts."""
    m = np.asarray(m, dtype=np.float64).reshape(4, 4)
    R = m[:3, :3]
    t = m[:3, 3]
    tr = float(np.trace(R))
    cos_theta = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    theta = float(np.arccos(cos_theta))
    if theta < 1e-8:
        phi = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        rho = t.copy()
    else:
        k = theta / (2.0 * np.sin(theta))
        phi = k * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        phi_norm2 = float(phi @ phi)
        # J⁻¹ = I - 0.5 [φ]× + ((1 - θ cot(θ/2) / 2) / θ²) [φ]×²
        half_cot = (theta * 0.5) / np.tan(theta * 0.5)
        c2 = (1.0 - half_cot) / phi_norm2
        cross1 = np.cross(phi, t)
        cross2 = np.cross(phi, cross1)
        rho = t - 0.5 * cross1 + c2 * cross2
    rho_norm2 = float(rho @ rho)
    phi_norm2 = float(phi @ phi)
    return float(np.sqrt(rho_norm2 + phi_norm2))


def se3_from_translation(t: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, 3] = np.asarray(t, dtype=np.float64)
    return out


def se3_from_yaw_translation(yaw_rad: float, t: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    c, s = float(np.cos(yaw_rad)), float(np.sin(yaw_rad))
    return np.array([
        [c, 0, s, t[0]],
        [0, 1, 0, t[1]],
        [-s, 0, c, t[2]],
        [0, 0, 0, 1],
    ], dtype=np.float64)
