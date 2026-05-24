"""Sim(3) pose-graph optimization on a bridged trajectory + loop-closure edges.

Backed by g2o (python binding). Each vertex is a `VertexSim3Expmap` storing
`Scw` (Sim3, camera-from-world); each edge is an `EdgeSim3` whose measurement
is the relative `Sba = Sbw * Saw.inverse()` predicted by some pair-pose
estimator. Three sources of edges feed the graph:

1. **Chain edges** — from `Trajectory.pairs` (`success=True`). The dominant
   constraint by count.
2. **Bridge edges** — synthesized from the bridged trajectory: for every
   pair of consecutive segments sharing the same `world_group`, the
   bridged poses already encode the cross-gap transform, so we extract
   `T_ba = inv(T_wc[fb]) @ T_wc[fa]` between the last frame of segment A
   and the first frame of segment B and turn it into an edge.
3. **Loop-closure edges** — from `loop_closures/<det>.json`. The whole
   reason PGO exists for this data.

Monocular convention: every measurement carries `s=1` (unit-norm `t`). Sim(3)
optimization is forced (over SE(3)) because the chain's accumulated scale
drifts away from the loop closure's unit-norm — only a per-node scale
variable can reconcile them.

Output: optimized `T_wc` matrices with the per-node scale absorbed into the
translation column (so positions reflect the scale correction, while the
rotation block stays pure). The discarded "scale" floats are reported in the
result dict if you need them.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import g2o
import numpy as np

from pose import RelativePose, Trajectory
from loop_closure import LoopClosure


# --------------------------------------------------------------------------- #
# Sim3 helpers
# --------------------------------------------------------------------------- #


def _sim3_from_T(T: np.ndarray, s: float = 1.0) -> g2o.Sim3:
    """Build a g2o.Sim3 from a 4x4 rigid transform + scalar scale."""
    return g2o.Sim3(T[:3, :3].copy(), T[:3, 3].copy(), float(s))


def _T_from_sim3(sim3: g2o.Sim3) -> tuple[np.ndarray, float]:
    """Convert g2o.Sim3 (any meaning — works for both Tcw and Twc) into a 4x4
    matrix with rotation in the upper-left and *scale absorbed into the
    translation column*. Returns (T_4x4, scale). The 3x3 block stays pure-
    rotation; the scale is reported separately so downstream code that wants
    to transform points correctly can multiply by it."""
    R = sim3.rotation().matrix()
    t = sim3.translation()
    s = sim3.scale()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t   # for Swc, t already has scale built into it; see optimize()
    return T, float(s)


# --------------------------------------------------------------------------- #
# Loop-closure dedup
# --------------------------------------------------------------------------- #


def dedupe_loop_closures(closures: Iterable[LoopClosure],
                         nms_window: int = 5) -> list[LoopClosure]:
    """Keep the strongest closure per (frame_a±w, frame_b±w) cluster.

    Loop-closure clusters (many edges describing the same physical revisit)
    are common — feeding all of them to PGO over-weights one constraint.
    Sort by `n_inliers_pose` descending, then greedily keep edges that fall
    outside the (frame_a, frame_b) neighborhood of any already-kept edge.
    """
    closures = sorted(closures, key=lambda c: -c.rp.n_inliers_pose)
    kept: list[LoopClosure] = []
    for c in closures:
        if any(abs(c.frame_a - k.frame_a) < nms_window and
               abs(c.frame_b - k.frame_b) < nms_window for k in kept):
            continue
        kept.append(c)
    return kept


# --------------------------------------------------------------------------- #
# Pose-graph build + optimize
# --------------------------------------------------------------------------- #


@dataclass
class PGOResult:
    """Outputs of `optimize_sim3`.  `poses_wc` is the new (N, 4, 4) pose array
    (NaN where the input was invalid), `scales` is the per-node optimized
    scale (1.0 where invalid)."""

    poses_wc: np.ndarray
    scales: np.ndarray
    initial_chi2: float
    final_chi2: float
    n_vertices: int
    n_chain_edges: int
    n_bridge_edges: int
    n_loop_edges: int


def _info_matrix(n_inliers: int) -> np.ndarray:
    """Translate inlier count into a 7x7 information matrix on Sim(3) tangent.
    Clipped to avoid pathological dominance of very-high-inlier edges."""
    w = float(np.clip(n_inliers / 100.0, 0.5, 5.0))
    return w * np.eye(7)


def _add_edge(optimizer: g2o.SparseOptimizer,
              v_a: g2o.VertexSim3Expmap, v_b: g2o.VertexSim3Expmap,
              T_ba: np.ndarray, info: np.ndarray,
              robust_delta: float | None) -> None:
    e = g2o.EdgeSim3()
    e.set_vertex(0, v_a)
    e.set_vertex(1, v_b)
    e.set_measurement(_sim3_from_T(T_ba, s=1.0))
    e.set_information(info)
    if robust_delta is not None:
        kernel = g2o.RobustKernelHuber()
        kernel.set_delta(robust_delta)
        e.set_robust_kernel(kernel)
    optimizer.add_edge(e)


def optimize_sim3(traj: Trajectory,
                  closures: list[LoopClosure],
                  *,
                  iterations: int = 30,
                  robust_delta: float | None = 2.6,
                  verbose: bool = False) -> PGOResult:
    """Build the Sim(3) pose graph from `traj` + `closures` and optimize.

    `traj` is assumed to be a *bridged* trajectory (its `extra["world_group"]`
    tells us which segments share a world frame and therefore need an
    explicit bridge edge to keep the optimizer from drifting them apart).
    """
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverSim3(g2o.LinearSolverEigenSim3())
    algorithm = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(algorithm)

    n = len(traj.frame_ids)
    valid = traj.valid
    poses = traj.poses_wc

    # ---- vertices: one per valid frame ----
    vertex_of: dict[int, g2o.VertexSim3Expmap] = {}
    first_valid = int(np.where(valid)[0][0])
    for i in range(n):
        if not valid[i]:
            continue
        # Vertex stores Scw = inverse(Twc).
        Tcw = np.linalg.inv(poses[i])
        v = g2o.VertexSim3Expmap()
        v.set_id(i)
        v.set_estimate(_sim3_from_T(Tcw, s=1.0))
        v.set_marginalized(False)
        v.set_fixed(i == first_valid)            # gauge fix
        optimizer.add_vertex(v)
        vertex_of[i] = v

    # ---- chain edges from traj.pairs ----
    n_chain = 0
    for p in traj.pairs:
        if not p.success:
            continue
        if p.frame_a not in vertex_of or p.frame_b not in vertex_of:
            continue
        # measurement Sba in cam-from-world chaining is: Sbw = Sba * Saw,
        # so the relative is just T_ba from the pair (R, t).
        T_ba = np.eye(4)
        T_ba[:3, :3] = p.R
        T_ba[:3, 3] = p.t
        _add_edge(optimizer, vertex_of[p.frame_a], vertex_of[p.frame_b],
                  T_ba, _info_matrix(p.n_inliers_pose), robust_delta)
        n_chain += 1

    # ---- bridge edges synthesized from world_group ----
    n_bridge = 0
    world_group = traj.extra.get("world_group", list(range(len(traj.segments))))
    for k in range(len(traj.segments) - 1):
        if world_group[k] != world_group[k + 1]:
            continue
        _s_a, e_a = traj.segments[k]
        s_b, _e_b = traj.segments[k + 1]
        fa, fb = e_a - 1, s_b
        if fa not in vertex_of or fb not in vertex_of:
            continue
        # T_ba derived from the (already-bridged) world poses.
        T_ba = np.linalg.inv(poses[fb]) @ poses[fa]
        # Bridge edges are less trustworthy than chain edges (single
        # geometry-verified frame across a gap), so give them moderate weight.
        info = 0.5 * np.eye(7)
        _add_edge(optimizer, vertex_of[fa], vertex_of[fb],
                  T_ba, info, robust_delta)
        n_bridge += 1

    # ---- loop-closure edges ----
    n_loop = 0
    for c in closures:
        if c.frame_a not in vertex_of or c.frame_b not in vertex_of:
            continue
        T_ba = np.eye(4)
        T_ba[:3, :3] = c.rp.R
        T_ba[:3, 3] = c.rp.t
        _add_edge(optimizer, vertex_of[c.frame_a], vertex_of[c.frame_b],
                  T_ba, _info_matrix(c.rp.n_inliers_pose), robust_delta)
        n_loop += 1

    # ---- solve ----
    optimizer.initialize_optimization()
    optimizer.set_verbose(verbose)
    optimizer.compute_active_errors()         # populate chi2 at the initial guess
    initial_chi2 = float(optimizer.chi2())
    optimizer.optimize(iterations)
    final_chi2 = float(optimizer.chi2())

    # ---- extract optimized world-from-camera poses + per-node scale ----
    new_poses = np.full_like(poses, np.nan)
    scales = np.ones(n, dtype=np.float64)
    for i, v in vertex_of.items():
        Swc = v.estimate().inverse()
        R = Swc.rotation().matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = Swc.translation()    # scale already baked into translation
        new_poses[i] = T
        scales[i] = float(Swc.scale())

    return PGOResult(
        poses_wc=new_poses,
        scales=scales,
        initial_chi2=initial_chi2,
        final_chi2=final_chi2,
        n_vertices=len(vertex_of),
        n_chain_edges=n_chain,
        n_bridge_edges=n_bridge,
        n_loop_edges=n_loop,
    )


def trajectory_with_optimized_poses(traj: Trajectory,
                                    result: PGOResult) -> Trajectory:
    """Return a copy of `traj` with poses_wc replaced by the PGO result, plus
    `extra` annotated with per-node scale and edge counts."""
    return Trajectory(
        detector=traj.detector,
        method=traj.method + "+sim3pgo",
        camera=traj.camera,
        frame_ids=list(traj.frame_ids),
        poses_wc=result.poses_wc,
        valid=traj.valid.copy(),
        segments=list(traj.segments),
        pairs=list(traj.pairs),
        extra={
            **traj.extra,
            "pgo_scales": result.scales.tolist(),
            "pgo_initial_chi2": result.initial_chi2,
            "pgo_final_chi2": result.final_chi2,
            "pgo_n_chain": result.n_chain_edges,
            "pgo_n_bridge": result.n_bridge_edges,
            "pgo_n_loop": result.n_loop_edges,
        },
    )
