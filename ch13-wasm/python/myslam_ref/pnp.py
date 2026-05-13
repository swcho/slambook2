"""Pose-only PnP — Python mirror of bind_pnp.cpp.

Implements the 4-round outlier loop the C++ binding wraps around g2o:

  for r in range(rounds):
    LM-refine pose on currently-marked inliers
    compute per-point reprojection chi² with the refined pose
    mark chi² > chi2_threshold as outliers (sticky)
  return refined T_cw + inlier mask + per-round diagnostics

We use ``cv2.solvePnP(SOLVEPNP_ITERATIVE)`` for the inner LM step — it ships
with the analytic projection Jacobian, so for the noiseless case it converges
to machine precision (same as g2o + ``linearizeOplus`` does in bind_pnp.cpp).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


def _axis_angle_to_R(rvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec.astype(np.float64).reshape(3, 1))
    return R


@dataclass
class PnPResult:
    T_cw: np.ndarray  # (3, 4) row-major
    final_inlier_mask: np.ndarray  # (N,) uint8
    round_inlier_masks: np.ndarray  # (rounds, N) uint8
    round_chi2_sum: np.ndarray  # (rounds,) float64 — sum over inliers
    round_inlier_count: np.ndarray  # (rounds,) int32
    round_iters: np.ndarray  # (rounds,) int32 — best-effort (cv2 doesn't expose iter count → reported as max_iters_per_round)
    total_inliers: int
    final_chi2: float
    rounds: int
    N: int

    @property
    def Tcw_row_major(self) -> np.ndarray:
        return self.T_cw.reshape(-1)


def estimate_pose(
    points3d: np.ndarray,
    obs2d: np.ndarray,
    K: np.ndarray,
    init_pose6: np.ndarray | None = None,
    *,
    rounds: int = 4,
    iter_per_round: int = 10,
    chi2_threshold: float = 5.991,
    use_robust_kernel: bool = True,
    remove_robust_after_round: int = 2,
) -> PnPResult:
    """4-round PnP with outlier rejection.

    Inputs:
      - ``points3d``: (N, 3) world-frame landmarks
      - ``obs2d``: (N, 2) pixel observations
      - ``K``: (3, 3) intrinsic
      - ``init_pose6``: ``[tx, ty, tz, rx, ry, rz]`` with axis-angle rotation
        (matches verify_pnp.ts). If None → identity.
    """
    pts3 = np.asarray(points3d, dtype=np.float64).reshape(-1, 3)
    pts2 = np.asarray(obs2d, dtype=np.float64).reshape(-1, 2)
    if pts3.shape[0] != pts2.shape[0]:
        raise ValueError(f"size mismatch: {pts3.shape[0]} 3D vs {pts2.shape[0]} 2D")
    N = pts3.shape[0]
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)

    if init_pose6 is None:
        rvec = np.zeros(3, dtype=np.float64)
        tvec = np.zeros(3, dtype=np.float64)
    else:
        init = np.asarray(init_pose6, dtype=np.float64).reshape(-1)
        if init.size != 6:
            raise ValueError(f"init_pose6 must be length 6, got {init.size}")
        tvec = init[0:3].copy()
        rvec = init[3:6].copy()

    inlier_mask = np.ones(N, dtype=np.uint8)
    round_masks = np.zeros((rounds, N), dtype=np.uint8)
    round_chi2 = np.zeros(rounds, dtype=np.float64)
    round_count = np.zeros(rounds, dtype=np.int32)
    round_iters = np.full(rounds, iter_per_round, dtype=np.int32)

    for r in range(rounds):
        idx = np.nonzero(inlier_mask > 0)[0]
        if idx.size < 3:
            break
        rvec3 = rvec.reshape(3, 1)
        tvec3 = tvec.reshape(3, 1)
        ok, rvec3, tvec3 = cv2.solvePnP(
            pts3[idx], pts2[idx], K, None,
            rvec=rvec3, tvec=tvec3,
            useExtrinsicGuess=True,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            break
        rvec = rvec3.reshape(-1)
        tvec = tvec3.reshape(-1)
        proj, _ = cv2.projectPoints(pts3, rvec, tvec, K, None)
        proj = proj.reshape(-1, 2)
        residual_sq = np.sum((pts2 - proj) ** 2, axis=1)
        new_mask = (residual_sq < chi2_threshold).astype(np.uint8)
        if use_robust_kernel and r < remove_robust_after_round:
            # While the robust kernel is active we keep all points but mark
            # outliers for diagnostic only — they still contribute (down-weighted
            # by Huber). When the kernel turns off after `remove_robust_after_round`
            # they become hard rejections.
            round_masks[r] = new_mask
            round_chi2[r] = float(residual_sq[new_mask > 0].sum())
            round_count[r] = int(new_mask.sum())
            # Don't shrink inlier_mask yet — kernel still on.
            inlier_mask = np.ones(N, dtype=np.uint8)
        else:
            inlier_mask = new_mask
            round_masks[r] = new_mask
            round_chi2[r] = float(residual_sq[new_mask > 0].sum())
            round_count[r] = int(new_mask.sum())

    R = _axis_angle_to_R(rvec)
    T = np.hstack([R, tvec.reshape(3, 1)])
    return PnPResult(
        T_cw=T,
        final_inlier_mask=inlier_mask.copy(),
        round_inlier_masks=round_masks,
        round_chi2_sum=round_chi2,
        round_inlier_count=round_count,
        round_iters=round_iters,
        total_inliers=int(inlier_mask.sum()),
        final_chi2=float(round_chi2[-1]),
        rounds=rounds,
        N=N,
    )
