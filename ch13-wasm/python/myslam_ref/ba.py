"""Bundle Adjustment — Python mirror of bind_ba.cpp.

Implements the same problem the C++ binding solves:

  minimize Σ ‖z_i − π(K · (T_cw_p_i · X_l_i))‖²

over a set of poses {T_cw_p} and landmarks {X_l}, with stereo extrinsics
``left_ext`` / ``right_ext`` applied per observation (matching ch13's
``EdgeProjection`` with separate left/right camera poses).

We use ``scipy.optimize.least_squares`` (Levenberg–Marquardt with sparse
Jacobian) so the inner step matches g2o's LM behavior. The adaptive chi²
threshold loop (``while inlier_ratio < 0.5: threshold *= 2``) is wrapped on
top, mirroring ``ch13/src/backend.cpp::Optimize``.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix


def _rvec_R(rvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    return R


def _R_rvec(R: np.ndarray) -> np.ndarray:
    r, _ = cv2.Rodrigues(R)
    return r.reshape(-1)


def _pose_params_from_T(T12: np.ndarray) -> np.ndarray:
    """``T12`` (12-flat or 3×4) → ``[tx, ty, tz, rx, ry, rz]`` (axis-angle)."""
    T = T12.reshape(3, 4)
    rvec = _R_rvec(T[:, :3])
    return np.concatenate([T[:, 3], rvec])


def _T_from_pose_params(p6: np.ndarray) -> np.ndarray:
    """``[tx, ty, tz, rx, ry, rz]`` → row-major 3×4 ``[R | t]``."""
    R = _rvec_R(p6[3:6])
    t = p6[0:3].reshape(3, 1)
    return np.hstack([R, t])


@dataclass
class BaResult:
    refined_poses12: np.ndarray  # (P, 12) row-major 3×4 per pose
    refined_landmarks3: np.ndarray  # (L, 3)
    per_edge_chi2_initial: np.ndarray  # (O,)
    per_edge_chi2_final: np.ndarray  # (O,)
    final_inlier_mask: np.ndarray  # (O,) uint8
    initial_chi2_sum: float
    final_chi2_sum: float
    iterations: int
    final_chi2_threshold: float
    adaptive_doublings: int
    final_inlier_count: int
    final_inlier_ratio: float
    P: int
    L: int
    O: int


def _project_one(K: np.ndarray, T_cw: np.ndarray, ext: np.ndarray, X_w: np.ndarray) -> np.ndarray:
    """Project ``X_w`` through pose ``T_cw`` then camera extrinsic ``ext`` then K."""
    # ext, T_cw: 3×4. We use full 4×4 internally for composition.
    Tcw4 = np.eye(4); Tcw4[:3, :] = T_cw
    Ext4 = np.eye(4); Ext4[:3, :] = ext
    P = K @ (Ext4 @ Tcw4)[:3, :]
    Xh = np.concatenate([X_w, [1.0]])
    uv_h = P @ Xh
    return uv_h[:2] / uv_h[2]


def _residuals(params: np.ndarray, P: int, L: int, obs: np.ndarray, fixed_idx: set[int], K: np.ndarray, left_ext: np.ndarray, right_ext: np.ndarray, init_poses12: np.ndarray) -> np.ndarray:
    """Pack residuals (z - z_hat) for the LM solver.

    Parameter layout:
      - 6 params per non-fixed pose (translation + axis-angle), totaling 6*(P - F)
      - 3 params per landmark
    Fixed poses come from ``init_poses12`` directly.
    """
    free_poses = [p for p in range(P) if p not in fixed_idx]
    n_free = len(free_poses)
    pose_params = params[: 6 * n_free].reshape(n_free, 6)
    lm_params = params[6 * n_free : 6 * n_free + 3 * L].reshape(L, 3)

    poses_T = [None] * P
    for p_idx, p in enumerate(free_poses):
        poses_T[p] = _T_from_pose_params(pose_params[p_idx])
    for p in fixed_idx:
        poses_T[p] = init_poses12[p].reshape(3, 4)

    res = np.empty(obs.shape[0] * 2)
    for i in range(obs.shape[0]):
        p_id = int(obs[i, 0]); l_id = int(obs[i, 1])
        u, v = obs[i, 2], obs[i, 3]
        is_left = obs[i, 4] >= 0.5
        ext = left_ext if is_left else right_ext
        uv_hat = _project_one(K, poses_T[p_id], ext, lm_params[l_id])
        res[2 * i + 0] = u - uv_hat[0]
        res[2 * i + 1] = v - uv_hat[1]
    return res


def _jac_sparsity(P: int, L: int, obs: np.ndarray, fixed_idx: set[int]) -> lil_matrix:
    free_poses = [p for p in range(P) if p not in fixed_idx]
    pose_col_of = {p: i for i, p in enumerate(free_poses)}
    n_free = len(free_poses)
    n_params = 6 * n_free + 3 * L
    n_res = 2 * obs.shape[0]
    m = lil_matrix((n_res, n_params), dtype=np.float64)
    for i in range(obs.shape[0]):
        p_id = int(obs[i, 0]); l_id = int(obs[i, 1])
        rows = [2 * i, 2 * i + 1]
        if p_id in pose_col_of:
            for c in range(6):
                m[rows[0], 6 * pose_col_of[p_id] + c] = 1
                m[rows[1], 6 * pose_col_of[p_id] + c] = 1
        for c in range(3):
            m[rows[0], 6 * n_free + 3 * l_id + c] = 1
            m[rows[1], 6 * n_free + 3 * l_id + c] = 1
    return m


def optimize(
    init_poses12: np.ndarray,
    init_landmarks3: np.ndarray,
    observations: np.ndarray,
    fixed_pose_indices: np.ndarray,
    K: np.ndarray,
    left_ext12: np.ndarray | None = None,
    right_ext12: np.ndarray | None = None,
    *,
    iterations: int = 20,
    chi2_init: float = 5.991,
    adaptive_rounds: int = 5,
    use_robust_kernel: bool = False,
) -> BaResult:
    """Bundle Adjustment with adaptive chi² inlier threshold.

    Inputs match bind_ba.cpp:
      - ``init_poses12``: (P, 12) row-major 3×4 per pose
      - ``init_landmarks3``: (L, 3)
      - ``observations``: (O, 5) with columns ``[pose_idx, lm_idx, u, v, is_left]``
      - ``fixed_pose_indices``: (F,) indices into the pose set (gauge fix)
      - ``K``: (3, 3) or length-9 row-major
      - ``left_ext12``/``right_ext12``: (3, 4) extrinsics — default to identity
    """
    init_poses12 = np.asarray(init_poses12, dtype=np.float64).reshape(-1, 12)
    init_landmarks3 = np.asarray(init_landmarks3, dtype=np.float64).reshape(-1, 3)
    P = init_poses12.shape[0]
    L = init_landmarks3.shape[0]
    obs = np.asarray(observations, dtype=np.float64).reshape(-1, 5)
    O = obs.shape[0]
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    left_ext = (np.asarray(left_ext12, dtype=np.float64).reshape(3, 4) if left_ext12 is not None
                else np.hstack([np.eye(3), np.zeros((3, 1))]))
    right_ext = (np.asarray(right_ext12, dtype=np.float64).reshape(3, 4) if right_ext12 is not None
                 else np.hstack([np.eye(3), np.zeros((3, 1))]))
    fixed_idx = {int(i) for i in np.asarray(fixed_pose_indices, dtype=np.int64).reshape(-1).tolist()}

    free_poses = [p for p in range(P) if p not in fixed_idx]
    n_free = len(free_poses)

    # Pack initial parameter vector.
    x0 = np.empty(6 * n_free + 3 * L)
    for i, p in enumerate(free_poses):
        x0[6 * i : 6 * (i + 1)] = _pose_params_from_T(init_poses12[p])
    x0[6 * n_free :] = init_landmarks3.reshape(-1)

    sparsity = _jac_sparsity(P, L, obs, fixed_idx)

    # Initial chi² (per-edge).
    r0 = _residuals(x0, P, L, obs, fixed_idx, K, left_ext, right_ext, init_poses12)
    per_edge_initial = (r0.reshape(-1, 2) ** 2).sum(axis=1)
    initial_chi2_sum = float(per_edge_initial.sum())

    chi2_thr = float(chi2_init)
    doublings = 0
    x = x0
    last_iters = 0
    inlier_mask = np.ones(O, dtype=np.uint8)
    while True:
        # Solve LM on currently active edges.
        active = np.nonzero(inlier_mask > 0)[0]
        if active.size < 1:
            break
        obs_active = obs[active]
        sub_sparsity = _jac_sparsity(P, L, obs_active, fixed_idx)
        result = least_squares(
            _residuals, x,
            args=(P, L, obs_active, fixed_idx, K, left_ext, right_ext, init_poses12),
            method="trf", jac_sparsity=sub_sparsity, max_nfev=iterations * 10, xtol=1e-12, ftol=1e-12,
        )
        x = result.x
        last_iters = int(result.nfev)
        # Re-evaluate per-edge chi² on ALL observations (sticky outlier semantics).
        r_full = _residuals(x, P, L, obs, fixed_idx, K, left_ext, right_ext, init_poses12)
        per_edge = (r_full.reshape(-1, 2) ** 2).sum(axis=1)
        new_mask = (per_edge < chi2_thr).astype(np.uint8)
        inlier_count = int(new_mask.sum())
        inlier_ratio = inlier_count / O if O else 1.0
        inlier_mask = new_mask
        # Adaptive threshold doubling — verify_ba.ts §D semantics.
        if inlier_ratio < 0.5 and doublings < adaptive_rounds:
            chi2_thr *= 2.0
            doublings += 1
            continue
        break

    # Unpack final params.
    refined_poses = init_poses12.copy()
    for p_idx, p in enumerate(free_poses):
        refined_poses[p] = _T_from_pose_params(x[6 * p_idx : 6 * (p_idx + 1)]).reshape(-1)
    refined_lms = x[6 * n_free :].reshape(L, 3)
    r_final = _residuals(x, P, L, obs, fixed_idx, K, left_ext, right_ext, init_poses12)
    per_edge_final = (r_final.reshape(-1, 2) ** 2).sum(axis=1)
    final_inlier_count = int(inlier_mask.sum())
    return BaResult(
        refined_poses12=refined_poses,
        refined_landmarks3=refined_lms,
        per_edge_chi2_initial=per_edge_initial,
        per_edge_chi2_final=per_edge_final,
        final_inlier_mask=inlier_mask,
        initial_chi2_sum=initial_chi2_sum,
        final_chi2_sum=float(per_edge_final.sum()),
        iterations=last_iters,
        final_chi2_threshold=chi2_thr,
        adaptive_doublings=doublings,
        final_inlier_count=final_inlier_count,
        final_inlier_ratio=final_inlier_count / O if O else 1.0,
        P=P, L=L, O=O,
    )
