"""External-library backends for the §4 PnP variant.

Goal: faithfully mirror ``bind_pnp.cpp`` 's g2o graph
(``VertexPoseSE3`` + ``EdgeReprojectionPoseOnly`` + ``RobustKernelHuber``) using
an off-the-shelf nonlinear optimizer library, so step08 can compare its result
to the three numpy-native variants.

Backend choice (per ``step08_pose_estimation.TODO.md`` §의사결정 보류):

  * **g2o-python (0.0.12)** — first attempt. macOS arm64 wheel currently
    segfaults the moment a non-pose vertex (``VertexPointXYZ``) is added to a
    ``SparseOptimizer``, and the Python binding for
    ``EdgeSE3ProjectXYZOnlyPose`` does NOT expose the ``Xw`` / ``fx`` / ``fy``
    / ``cx`` / ``cy`` C++ public fields. Subclassing ``BaseUnaryEdge`` is also
    blocked (``No constructor defined``). Variant disabled, see ``inner_solve_g2o``.

  * **gtsam (4.2.1)** — works. We use ``GenericProjectionFactorCal3_S2`` and
    pin each landmark with a near-zero variance ``PriorFactorPoint3`` so only
    the pose ``Pose3`` is effectively optimized — same graph topology as the
    g2o pose-only edge, just expressed in gtsam's factor language.
    ``noiseModel.Robust.Create(mEstimator.Huber.Create(δ), …)`` gives us the
    Huber kernel ``bind_pnp.cpp`` adds via ``RobustKernelHuber``.

Either backend's solver is wrapped to match the inner-solver signature
``_run_4round_loop`` expects (see ``myslam_ref/pnp.py``).
"""

from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np

from myslam_ref.pnp import _InnerSolveOutput


# --------------------------------------------------------------------------
# g2o-python backend — currently disabled.
# --------------------------------------------------------------------------


def inner_solve_g2o(g2o_mod: Any):
    """Build an inner-solver callable for g2o-python.

    Disabled because g2o-python 0.0.12 crashes when adding ``VertexPointXYZ``
    on macOS arm64 (and ``EdgeSE3ProjectXYZOnlyPose`` doesn't expose ``Xw`` to
    Python anyway). Kept as a stub so the dispatcher logic remains symmetric;
    raises ``NotImplementedError`` at solve time so the testbench can fall
    through to gtsam.
    """
    del g2o_mod  # unused

    def solve(
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
        raise NotImplementedError(
            "g2o-python (0.0.12) Python binding does not expose the "
            "EdgeSE3ProjectXYZOnlyPose intrinsics/world-point fields, and "
            "adding a VertexPointXYZ to a SparseOptimizer segfaults on the "
            "macOS arm64 wheel. Use gtsam instead."
        )

    return solve


# --------------------------------------------------------------------------
# gtsam backend.
# --------------------------------------------------------------------------


def _gtsam_pose3_from_Tcw(gtsam_mod: Any, rvec: np.ndarray, tvec: np.ndarray):
    """Build ``gtsam.Pose3`` (= T_wc) from our T_cw rvec/tvec.

    gtsam의 ``GenericProjectionFactor`` 는 ``PinholeCamera::project(point, pose)`` 를
    호출하는데, 이 때 ``pose`` 는 **카메라가 월드 좌표계에서 갖는 자세**
    (T_wc = T_cw⁻¹). 우리 pipeline 의 T_cw 와 부호 반대이므로 변환 필요.
    """
    R_cw, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    t_cw = np.asarray(tvec, dtype=np.float64).reshape(3)
    R_wc = R_cw.T
    t_wc = -R_cw.T @ t_cw
    return gtsam_mod.Pose3(gtsam_mod.Rot3(R_wc.astype(np.float64)), t_wc)


def _Tcw_rvec_tvec_from_pose3(pose3: Any) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(rvec, tvec)`` of T_cw from gtsam's T_wc Pose3."""
    R_wc = np.asarray(pose3.rotation().matrix(), dtype=np.float64)
    t_wc = np.asarray(pose3.translation(), dtype=np.float64).reshape(3)
    R_cw = R_wc.T
    t_cw = -R_wc.T @ t_wc
    rvec, _ = cv2.Rodrigues(R_cw)
    return rvec.reshape(-1).astype(np.float64), t_cw


def inner_solve_gtsam(gtsam_mod: Any):
    """Build an inner-solver callable bound to a gtsam module instance.

    Graph:
      * one ``Pose3`` variable for T_cw
      * N ``Point3`` variables for the landmarks, pinned by a strong
        ``PriorFactorPoint3`` (effectively ``setFixed(true)`` in g2o)
      * N ``GenericProjectionFactorCal3_S2`` factors carrying the pixel
        observation, optionally wrapped in ``Robust(Huber)``

    LM optimizer is ``LevenbergMarquardtOptimizer`` with the
    ``iter_per_round`` cap as the LM iteration ceiling.
    """

    def solve(
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
        pts3 = np.asarray(pts3_in, dtype=np.float64).reshape(-1, 3)
        pts2 = np.asarray(pts2_in, dtype=np.float64).reshape(-1, 2)
        K = np.asarray(K, dtype=np.float64).reshape(3, 3)

        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])
        cal = gtsam_mod.Cal3_S2(fx, fy, 0.0, cx, cy)

        pose_key = gtsam_mod.symbol("x", 0)
        init_pose = _gtsam_pose3_from_Tcw(gtsam_mod, rvec_init, tvec_init)

        graph = gtsam_mod.NonlinearFactorGraph()
        values = gtsam_mod.Values()
        values.insert(pose_key, init_pose)

        # 1-pixel isotropic noise. bind_pnp.cpp uses Identity information
        # matrix (= σ = 1 px after Cholesky), so this matches.
        pixel_noise = gtsam_mod.noiseModel.Isotropic.Sigma(2, 1.0)
        if use_robust_kernel:
            delta = math.sqrt(max(chi2_threshold, 1e-12))
            obs_noise = gtsam_mod.noiseModel.Robust.Create(
                gtsam_mod.noiseModel.mEstimator.Huber.Create(delta),
                pixel_noise,
            )
        else:
            obs_noise = pixel_noise

        # Pin landmarks: σ = 1e-9 m → ~zero variance, so the LM step on Point3
        # is effectively zero. Equivalent to g2o's setFixed(true).
        pin_noise = gtsam_mod.noiseModel.Isotropic.Sigma(3, 1e-9)

        for i, (Pw, uv) in enumerate(zip(pts3, pts2, strict=True)):
            lm_key = gtsam_mod.symbol("l", i)
            values.insert(lm_key, Pw.copy())
            graph.add(
                gtsam_mod.GenericProjectionFactorCal3_S2(
                    uv.copy(),
                    obs_noise,
                    pose_key,
                    lm_key,
                    cal,
                )
            )
            graph.add(gtsam_mod.PriorFactorPoint3(lm_key, Pw.copy(), pin_noise))

        params = gtsam_mod.LevenbergMarquardtParams()
        params.setMaxIterations(int(iter_per_round))
        params.setVerbosityLM("SILENT")
        optimizer = gtsam_mod.LevenbergMarquardtOptimizer(graph, values, params)
        result = optimizer.optimize()
        T_opt = result.atPose3(pose_key)
        rvec_out, tvec_out = _Tcw_rvec_tvec_from_pose3(T_opt)
        iters = int(optimizer.iterations())
        return _InnerSolveOutput(rvec=rvec_out, tvec=tvec_out, iters=iters)

    return solve
