"""Hand-written SE(3) Levenberg-Marquardt for pose-only PnP.

`bind_pnp.cpp` 의 g2o + RobustKernelHuber inner solver 를 numpy 로 직접
다시 쓴 구현. 학습용 — g2o 가 ``_jacobianOplusXi`` + ``oplusImpl`` + LM 댐핑 +
``RobustKernel::computeRobustWeights`` 를 어떻게 연결하는지 한 파일에서 확인.

흐름 (한 라운드 = ``iter_per_round`` 회 inner LM):

  for it in range(iter_per_round):
      r = z_obs - π(T · Pw)                       # (2N,)
      J = stack(_pose_jacobian(T, Pw_i, K))       # (2N, 6)
      W = diag(_huber_weight(r²_i, δ²))           # IRLS — kernel-active 때만
      (Jᵀ W J + λI) δ = Jᵀ W r                    # normal equation
      T' = exp(δ) · T  (left update)
      if cost(T') < cost(T): accept, λ /= 10
      else:                  reject, λ *= 10

초기 λ = 1e-3 · max(diag(JᵀJ)) — g2o ``OptimizationAlgorithmLevenberg`` 의 휴리스틱.
"""

from __future__ import annotations

import cv2
import numpy as np

from myslam_ref.pnp import (
    _InnerSolveOutput,
    _huber_weight,
    _pose_jacobian,
    _project,
    _se3_left_update,
)


def _T4_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """``[R t; 0 1]`` (4×4) — internal LM 은 SE(3) full 4×4 가 편하다."""
    T = np.eye(4, dtype=np.float64)
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def _rvec_tvec_from_T4(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return rvec.reshape(-1).astype(np.float64), T[:3, 3].astype(np.float64).copy()


def _residual_and_jacobian(
    T: np.ndarray,
    pts3: np.ndarray,
    pts2: np.ndarray,
    K: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(r, J)`` for the current inlier set.

    ``r ∈ ℝ^{2N}`` is ``measurement - projection`` (g2o convention).
    ``J ∈ ℝ^{2N×6}`` stacks ``_pose_jacobian`` per row — note this is
    ``∂r/∂xi`` already (sign baked in by bind_pnp.cpp::linearizeOplus).
    """
    proj, _ = _project(T, pts3, K)
    r = (pts2 - proj).reshape(-1)
    N = pts3.shape[0]
    J = np.empty((2 * N, 6), dtype=np.float64)
    for i in range(N):
        J[2 * i : 2 * i + 2, :] = _pose_jacobian(T, pts3[i], K)
    return r, J


def _huber_cost(r: np.ndarray, delta2: float, use_kernel: bool) -> float:
    """Robust cost ρ(r²). Plain SSR when kernel off, Huber otherwise."""
    pairs = r.reshape(-1, 2)
    r2 = np.sum(pairs * pairs, axis=1)
    if not use_kernel:
        return float(r2.sum())
    delta = float(np.sqrt(delta2))
    cost = np.where(r2 <= delta2, r2, 2.0 * delta * np.sqrt(np.maximum(r2, 0.0)) - delta2)
    return float(cost.sum())


def _solve_lm_step(
    JtWJ: np.ndarray,
    JtWr: np.ndarray,
    lam: float,
) -> np.ndarray:
    """Levenberg damping: ``(H + λ·diag(H)) δ = g`` — Marquardt 변종.

    g2o ``OptimizationAlgorithmLevenberg`` 도 ``λ·diag(H)`` 를 쓴다
    (plain ``λ·I`` 보다 스케일 invariance 가 좋음).
    """
    H = JtWJ + lam * np.diag(np.maximum(np.diag(JtWJ), 1e-12))
    try:
        delta = np.linalg.solve(H, JtWr)
    except np.linalg.LinAlgError:
        # 매우 드물게 H 가 거의 singular — 작은 ridge 를 더해 재시도.
        H = JtWJ + (lam + 1e-6) * np.eye(JtWJ.shape[0])
        delta = np.linalg.solve(H, JtWr)
    return delta


def inner_solve_handwritten(
    pts3_in: np.ndarray,
    pts2_in: np.ndarray,
    K: np.ndarray,
    rvec_init: np.ndarray,
    tvec_init: np.ndarray,
    *,
    iter_per_round: int,
    chi2_threshold: float,
    use_robust_kernel: bool,
) -> _InnerSolveOutput:
    """SE(3) LM with optional Huber IRLS — pluggable into ``_run_4round_loop``."""
    pts3 = np.asarray(pts3_in, dtype=np.float64).reshape(-1, 3)
    pts2 = np.asarray(pts2_in, dtype=np.float64).reshape(-1, 2)
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    delta2 = float(max(chi2_threshold, 1e-12))

    T = _T4_from_rvec_tvec(rvec_init, tvec_init)
    r, J = _residual_and_jacobian(T, pts3, pts2, K)
    # Initial λ — g2o LM 의 초기화 휴리스틱.
    JtJ_diag = np.einsum("ij,ij->j", J, J)
    lam = 1e-3 * float(np.max(JtJ_diag)) if JtJ_diag.size else 1e-3
    cost = _huber_cost(r, delta2, use_robust_kernel)

    iters_used = 0
    for _ in range(int(iter_per_round)):
        # IRLS weights — kernel-active 일 때만 down-weight.
        if use_robust_kernel:
            pairs = r.reshape(-1, 2)
            r2 = np.sum(pairs * pairs, axis=1)
            w_edge = _huber_weight(r2, delta2)
            # 2N 다이아고날 — 각 edge 의 두 row 가 같은 weight.
            w = np.repeat(w_edge, 2)
        else:
            w = np.ones(r.shape[0], dtype=np.float64)

        Jw = J * w[:, None]
        H = J.T @ Jw  # JᵀWJ
        # Gauss-Newton normal equation: H δ = -JᵀW r (g2o 도 b = -J^T Ω error 로 풂).
        # `_pose_jacobian` 은 ∂r/∂xi 의 부호로 들어와 있으므로 여기서 마이너스 한 번만.
        b = -(J.T @ (w * r))

        # Try a step; reject + grow λ if cost doesn't drop.
        delta = _solve_lm_step(H, b, lam)
        T_trial = _se3_left_update(T, delta)
        r_trial, J_trial = _residual_and_jacobian(T_trial, pts3, pts2, K)
        cost_trial = _huber_cost(r_trial, delta2, use_robust_kernel)

        iters_used += 1
        if cost_trial < cost:
            T = T_trial
            r, J = r_trial, J_trial
            cost = cost_trial
            lam = max(lam / 10.0, 1e-12)
            if float(np.linalg.norm(delta)) < 1e-9:
                break
        else:
            lam = min(lam * 10.0, 1e12)
            if lam >= 1e12:
                break

    rvec_out, tvec_out = _rvec_tvec_from_T4(T)
    return _InnerSolveOutput(rvec=rvec_out, tvec=tvec_out, iters=iters_used)
