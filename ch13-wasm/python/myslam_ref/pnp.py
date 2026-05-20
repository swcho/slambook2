"""Pose-only PnP — Python mirror of bind_pnp.cpp.

4-round outlier loop (`bind_pnp.cpp::estimatePose` 의 frontend.cpp 미러):

  for r in range(rounds):
      LM-refine pose on currently-marked inliers (inner solver 만 갈아끼움)
      compute per-edge reprojection chi² with the refined pose
      mark chi² > chi2_threshold as outliers (sticky)
      starting from `remove_robust_after_round`, kernel 비활성화로 hard reject
  return refined T_cw + inlier mask + per-round diagnostics

inner LM 솔버는 4 가지 변종을 제공한다:

  1. estimate_pose_cv2          — cv2.solvePnP(SOLVEPNP_ITERATIVE).
                                   Huber 가 진짜가 아닌 mask-freeze 근사.
  2. estimate_pose_scipy        — scipy.optimize.least_squares(loss='huber').
                                   scipy 내장 진짜 Huber.
  3. estimate_pose_handwritten  — myslam_ref.pnp_lm 의 SE(3) LM + IRLS.
                                   g2o RobustKernelHuber 와 동일한 weight.
  4. estimate_pose_g2o          — g2o-python (또는 gtsam) 라이브러리.
                                   bind_pnp.cpp 와 가장 직설적인 매핑.

공통 시그니처/반환은 ``PnPResult`` 로 통일. 기존 ``estimate_pose`` 는
``estimate_pose_cv2`` 의 alias 로 유지 (step01~13 스크립트 호환).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import cv2
import numpy as np


# --------------------------------------------------------------------------
# SE(3) helpers — bind_pnp.cpp 의 VertexPoseSE3::oplusImpl 미러.
# --------------------------------------------------------------------------


def _axis_angle_to_R(rvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec.astype(np.float64).reshape(3, 1))
    return R


def _se3_left_update(T: np.ndarray, xi: np.ndarray) -> np.ndarray:
    """Left-multiply a 4×4 SE(3) by exp(xi) with xi = [ρ; φ] (g2o 관례).

    Mirrors ``bind_pnp.cpp::VertexPoseSE3::oplusImpl``:
      translation = xi.head(3), rotation = xi.tail(3) (axis-angle),
      ``_estimate = dT * _estimate``.
    """
    rho = np.asarray(xi[0:3], dtype=np.float64).reshape(3)
    phi = np.asarray(xi[3:6], dtype=np.float64).reshape(3)
    phi_norm = float(np.linalg.norm(phi))
    if phi_norm > 1e-12:
        axis = phi / phi_norm
    else:
        axis = np.array([0.0, 0.0, 1.0])
    R_delta = _axis_angle_to_R(axis * phi_norm)
    dT = np.eye(4, dtype=np.float64)
    dT[:3, :3] = R_delta
    dT[:3, 3] = rho
    T4 = np.eye(4, dtype=np.float64)
    T4[:3, :] = T[:3, :]
    return (dT @ T4).astype(np.float64)


def _T34_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """``[R | t]`` (3×4) from axis-angle + translation."""
    T = np.zeros((3, 4), dtype=np.float64)
    T[:, :3] = _axis_angle_to_R(rvec)
    T[:, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def _project(T: np.ndarray, pts3: np.ndarray, K: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """``(uv, chi²)`` for every world point under pose T (3×4 or 4×4).

    chi² is the squared 2D pixel residual against an implicit zero observation;
    callers subtract their measurement first. We return projections only.
    """
    R = T[:3, :3]
    t = T[:3, 3]
    Pc = pts3 @ R.T + t
    z = Pc[:, 2]
    uv = np.empty((pts3.shape[0], 2), dtype=np.float64)
    uv[:, 0] = K[0, 0] * Pc[:, 0] / z + K[0, 2]
    uv[:, 1] = K[1, 1] * Pc[:, 1] / z + K[1, 2]
    return uv, Pc


def _pose_jacobian(T: np.ndarray, Pw: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Per-point ``(2, 6)`` reprojection Jacobian.

    Direct port of ``bind_pnp.cpp::EdgeReprojectionPoseOnly::linearizeOplus``
    (lines 85–106). Layout matches g2o: ``∂error/∂[ρ φ]``. Note that the C++
    code follows g2o convention where ``error = measurement - projection`` and
    the Jacobian is ``-∂projection/∂xi`` — we reproduce the exact entries so
    a residual ``r = z_obs - π(T·Pw)`` paired with this J gives the correct
    normal equations.
    """
    R = T[:3, :3]
    t = T[:3, 3]
    Pc = R @ Pw + t
    X, Y, Z = float(Pc[0]), float(Pc[1]), float(Pc[2])
    Z2 = Z * Z
    fx, fy = float(K[0, 0]), float(K[1, 1])
    J = np.empty((2, 6), dtype=np.float64)
    J[0, 0] = -fx / Z
    J[0, 1] = 0.0
    J[0, 2] = fx * X / Z2
    J[0, 3] = fx * X * Y / Z2
    J[0, 4] = -fx - fx * X * X / Z2
    J[0, 5] = fx * Y / Z
    J[1, 0] = 0.0
    J[1, 1] = -fy / Z
    J[1, 2] = fy * Y / Z2
    J[1, 3] = fy + fy * Y * Y / Z2
    J[1, 4] = -fy * X * Y / Z2
    J[1, 5] = -fy * X / Z
    return J


def _huber_weight(r2: np.ndarray, delta2: float) -> np.ndarray:
    """g2o ``RobustKernelHuber`` 의 IRLS weight.

    ρ(s) = s if s ≤ δ², else 2δ√s - δ². Effective IRLS weight is
    ``w = 1 if r² ≤ δ² else δ/|r|`` (so that the weighted residual r·√w has
    the same gradient as the kernel-warped one).
    """
    r2 = np.asarray(r2, dtype=np.float64)
    delta = float(np.sqrt(delta2))
    w = np.ones_like(r2)
    mask = r2 > delta2
    if np.any(mask):
        r_abs = np.sqrt(r2[mask])
        w[mask] = delta / np.maximum(r_abs, 1e-30)
    return w


# --------------------------------------------------------------------------
# Result struct — common across all 4 variants.
# --------------------------------------------------------------------------


@dataclass
class PnPResult:
    T_cw: np.ndarray  # (3, 4) row-major
    final_inlier_mask: np.ndarray  # (N,) uint8
    round_inlier_masks: np.ndarray  # (rounds, N) uint8
    round_chi2_sum: np.ndarray  # (rounds,) float64 — sum over inliers
    round_inlier_count: np.ndarray  # (rounds,) int32
    round_iters: np.ndarray  # (rounds,) int32
    total_inliers: int
    final_chi2: float
    rounds: int
    N: int

    @property
    def Tcw_row_major(self) -> np.ndarray:
        return self.T_cw.reshape(-1)


# --------------------------------------------------------------------------
# Shared 4-round driver — inner_solve_fn 만 갈아끼우면 변종이 공유.
# --------------------------------------------------------------------------


@dataclass
class _InnerSolveOutput:
    rvec: np.ndarray  # (3,) axis-angle
    tvec: np.ndarray  # (3,)
    iters: int  # informational — best-effort iteration count


# inner_solve_fn signature:
#   fn(pts3_in, pts2_in, K, rvec_init, tvec_init, *, iter_per_round,
#      chi2_threshold, use_robust_kernel) -> _InnerSolveOutput
InnerSolveFn = Callable[..., _InnerSolveOutput]


def _run_4round_loop(
    inner_solve: InnerSolveFn,
    *,
    points3d: np.ndarray,
    obs2d: np.ndarray,
    K: np.ndarray,
    init_pose6: np.ndarray | None,
    rounds: int,
    iter_per_round: int,
    chi2_threshold: float,
    use_robust_kernel: bool,
    remove_robust_after_round: int,
) -> PnPResult:
    """Outlier-loop driver that mirrors ``bind_pnp.cpp::estimatePose`` rounds.

    Per round:
      * pick currently-inlier indices (sticky from previous rounds)
      * call ``inner_solve`` to LM-refine pose on those points
      * project ALL N points with the refined pose and compute chi² per edge
      * if kernel is active this round (round < remove_robust_after_round),
        record the mask for diagnostics but do NOT shrink the inlier set —
        the kernel itself down-weighted outliers in the inner LM. Once the
        kernel is dropped, chi² > threshold becomes a hard rejection.

    Returns a unified ``PnPResult``.
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
        kernel_active = use_robust_kernel and r < remove_robust_after_round
        out = inner_solve(
            pts3[idx],
            pts2[idx],
            K,
            rvec,
            tvec,
            iter_per_round=iter_per_round,
            chi2_threshold=chi2_threshold,
            use_robust_kernel=kernel_active,
        )
        rvec = np.asarray(out.rvec, dtype=np.float64).reshape(3)
        tvec = np.asarray(out.tvec, dtype=np.float64).reshape(3)
        round_iters[r] = int(out.iters)

        proj, _ = cv2.projectPoints(pts3, rvec, tvec, K, None)
        residual_sq = np.sum((pts2 - proj.reshape(-1, 2)) ** 2, axis=1)
        new_mask = (residual_sq < chi2_threshold).astype(np.uint8)
        round_masks[r] = new_mask
        round_chi2[r] = float(residual_sq[new_mask > 0].sum())
        round_count[r] = int(new_mask.sum())
        if not kernel_active:
            inlier_mask = new_mask
        # else: kernel still on → all points stay in the LM, mask is diagnostic.

    T = _T34_from_rvec_tvec(rvec, tvec)
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


# --------------------------------------------------------------------------
# Variant 1 — cv2.solvePnP(SOLVEPNP_ITERATIVE) baseline.
# --------------------------------------------------------------------------


def _inner_solve_cv2(
    pts3_in: np.ndarray,
    pts2_in: np.ndarray,
    K: np.ndarray,
    rvec_init: np.ndarray,
    tvec_init: np.ndarray,
    *,
    iter_per_round: int,
    chi2_threshold: float,  # noqa: ARG001 — Huber 가 진짜가 아니라 미사용
    use_robust_kernel: bool,  # noqa: ARG001 — cv2 변종은 mask-freeze 근사라 무시
) -> _InnerSolveOutput:
    rvec3 = rvec_init.astype(np.float64).reshape(3, 1)
    tvec3 = tvec_init.astype(np.float64).reshape(3, 1)
    ok, rvec3, tvec3 = cv2.solvePnP(
        pts3_in, pts2_in, K, None,
        rvec=rvec3, tvec=tvec3,
        useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return _InnerSolveOutput(rvec=rvec_init.copy(), tvec=tvec_init.copy(), iters=0)
    return _InnerSolveOutput(
        rvec=rvec3.reshape(-1).copy(),
        tvec=tvec3.reshape(-1).copy(),
        iters=int(iter_per_round),
    )


def estimate_pose_cv2(
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
    """cv2 baseline — ``cv2.solvePnP(SOLVEPNP_ITERATIVE)`` inner LM.

    Note: cv2 LM does NOT support Huber. 4-round 루프는 mask-freeze 근사로
    "kernel-on 라운드에서는 inlier 집합을 줄이지 않고, kernel-off 라운드에서만
    hard-reject" 한다. 진짜 IRLS Huber 가 필요하면 ``estimate_pose_handwritten``
    또는 ``estimate_pose_scipy`` 를 쓸 것.
    """
    return _run_4round_loop(
        _inner_solve_cv2,
        points3d=points3d,
        obs2d=obs2d,
        K=K,
        init_pose6=init_pose6,
        rounds=rounds,
        iter_per_round=iter_per_round,
        chi2_threshold=chi2_threshold,
        use_robust_kernel=use_robust_kernel,
        remove_robust_after_round=remove_robust_after_round,
    )


# --------------------------------------------------------------------------
# Variant 2 — scipy.optimize.least_squares(loss='huber') inner LM.
# --------------------------------------------------------------------------


def _inner_solve_scipy(
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
    from scipy.optimize import least_squares

    # x = [tx, ty, tz, rx, ry, rz] — same packing as init_pose6.
    x0 = np.concatenate([tvec_init.reshape(3), rvec_init.reshape(3)]).astype(np.float64)
    K_arr = np.asarray(K, dtype=np.float64)
    pts3 = np.asarray(pts3_in, dtype=np.float64).reshape(-1, 3)
    pts2 = np.asarray(pts2_in, dtype=np.float64).reshape(-1, 2)

    def residual(x: np.ndarray) -> np.ndarray:
        T = _T34_from_rvec_tvec(x[3:6], x[0:3])
        proj, _ = _project(T, pts3, K_arr)
        return (pts2 - proj).reshape(-1)

    def jac(x: np.ndarray) -> np.ndarray:
        T = _T34_from_rvec_tvec(x[3:6], x[0:3])
        K2 = K_arr
        N = pts3.shape[0]
        J = np.zeros((2 * N, 6), dtype=np.float64)
        for i in range(N):
            # residual = measurement - projection ⇒ ∂r/∂xi = -∂π/∂xi.
            # bind_pnp.cpp 의 _jacobianOplusXi 는 이미 ∂(measurement-π)/∂xi
            # 부호로 적혀 있어서 그대로 사용하면 sign convention 이 일치.
            J[2 * i : 2 * i + 2, :] = _pose_jacobian(T, pts3[i], K2)
        return J

    f_scale = float(np.sqrt(max(chi2_threshold, 1e-12)))
    loss = "huber" if use_robust_kernel else "linear"
    max_nfev = max(1, int(iter_per_round) * 6)
    try:
        result = least_squares(
            residual,
            x0,
            jac=jac,
            method="trf",
            loss=loss,
            f_scale=f_scale,
            max_nfev=max_nfev,
        )
    except Exception:
        return _InnerSolveOutput(rvec=rvec_init.copy(), tvec=tvec_init.copy(), iters=0)
    x = result.x
    # nfev → equivalent LM iters (scipy 가 정확한 iter 카운트를 노출하지 않음).
    iters = int(max(1, round(result.nfev / 6)))
    return _InnerSolveOutput(
        rvec=x[3:6].copy(),
        tvec=x[0:3].copy(),
        iters=iters,
    )


def estimate_pose_scipy(
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
    """scipy variant — ``least_squares(loss='huber')`` 진짜 Huber inner LM.

    kernel-active 라운드는 ``loss='huber', f_scale=√chi2_threshold`` 로 호출하고,
    ``remove_robust_after_round`` 이후 라운드는 ``loss='linear'`` 로 전환.
    """
    return _run_4round_loop(
        _inner_solve_scipy,
        points3d=points3d,
        obs2d=obs2d,
        K=K,
        init_pose6=init_pose6,
        rounds=rounds,
        iter_per_round=iter_per_round,
        chi2_threshold=chi2_threshold,
        use_robust_kernel=use_robust_kernel,
        remove_robust_after_round=remove_robust_after_round,
    )


# --------------------------------------------------------------------------
# Variant 3 — handwritten SE(3) LM (myslam_ref.pnp_lm 에 위임).
# --------------------------------------------------------------------------


def estimate_pose_handwritten(
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
    """Hand-written SE(3) LM + IRLS Huber (학습용)."""
    from myslam_ref.pnp_lm import inner_solve_handwritten

    return _run_4round_loop(
        inner_solve_handwritten,
        points3d=points3d,
        obs2d=obs2d,
        K=K,
        init_pose6=init_pose6,
        rounds=rounds,
        iter_per_round=iter_per_round,
        chi2_threshold=chi2_threshold,
        use_robust_kernel=use_robust_kernel,
        remove_robust_after_round=remove_robust_after_round,
    )


# --------------------------------------------------------------------------
# Variant 4 — g2o/gtsam backend (optional dependency).
# --------------------------------------------------------------------------


def _g2o_available() -> tuple[str, object] | None:
    """Return ``(backend_name, module)`` or ``None`` if neither library is installed.

    Tries **gtsam first**: ``g2o-python`` 0.0.12 의 macOS arm64 wheel 은
    ``VertexPointXYZ`` 추가 시 segfault 가 발생하고
    ``EdgeSE3ProjectXYZOnlyPose`` 의 ``Xw`` / 내참수 필드를 Python 에 노출하지
    않아 사실상 사용 불가. ``_pnp_g2o.inner_solve_g2o`` 의 stub 이 호출 시
    ``NotImplementedError`` 를 던지므로, 다른 환경에서 g2o-python 이 fix 되어
    있다 해도 이쪽 경로로는 동작하지 않는다. 그 환경에서는 직접 dispatcher
    를 손봐서 g2o 백엔드를 활성화할 것.
    """
    try:
        import gtsam as _gtsam  # type: ignore  # noqa: I001

        return ("gtsam", _gtsam)
    except Exception:
        pass
    try:
        import g2o as _g2o  # type: ignore  # noqa: I001

        return ("g2o", _g2o)
    except Exception:
        pass
    return None


def estimate_pose_g2o(
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
    """g2o-python (or gtsam) variant. Raises ``ImportError`` if unavailable.

    Caller (testbench) catches ``ImportError`` and reports N/A for this variant.
    """
    backend = _g2o_available()
    if backend is None:
        raise ImportError(
            "Neither `g2o-python` nor `gtsam` is installed — estimate_pose_g2o unavailable. "
            "Try `uv add g2o-python` (or `uv add gtsam`) inside ch13-wasm/python."
        )
    name, mod = backend
    if name == "g2o":
        from myslam_ref._pnp_g2o import inner_solve_g2o

        inner = inner_solve_g2o(mod)
    else:
        from myslam_ref._pnp_g2o import inner_solve_gtsam

        inner = inner_solve_gtsam(mod)
    return _run_4round_loop(
        inner,
        points3d=points3d,
        obs2d=obs2d,
        K=K,
        init_pose6=init_pose6,
        rounds=rounds,
        iter_per_round=iter_per_round,
        chi2_threshold=chi2_threshold,
        use_robust_kernel=use_robust_kernel,
        remove_robust_after_round=remove_robust_after_round,
    )


# --------------------------------------------------------------------------
# Backwards-compatible alias — keeps step01~13 + tests/test_smoke.py working.
# --------------------------------------------------------------------------

estimate_pose = estimate_pose_cv2
