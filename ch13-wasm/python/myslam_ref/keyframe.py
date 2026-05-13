"""Keyframe decision policies — Python mirror of src/steps/Step09_KeyframeDecision.

Three policies are exercised in the UI:

- ``inlier_threshold``: insert a KF when the current frame's tracked-inlier
  count drops below ``min_inliers``. This is the ch13 ``Frontend::InsertKeyframe``
  branch.
- ``frame_distance``: insert a KF when SE(3) log-norm distance from the last
  KF exceeds ``min_distance_m``.
- ``fixed_interval``: insert a KF every ``interval`` frames.

Each policy returns a bool + the metric value, so the step script can compare
side-by-side.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from .se3 import se3_log_norm


class Policy(str, Enum):
    INLIER_THRESHOLD = "inlier-threshold"
    FRAME_DISTANCE = "frame-distance"
    FIXED_INTERVAL = "fixed-interval"


@dataclass
class KeyframeDecision:
    insert: bool
    policy: Policy
    metric: float
    reason: str


def decide_inlier_threshold(tracked_inliers: int, *, min_inliers: int = 80) -> KeyframeDecision:
    insert = tracked_inliers < min_inliers
    return KeyframeDecision(
        insert=insert,
        policy=Policy.INLIER_THRESHOLD,
        metric=float(tracked_inliers),
        reason=f"inliers={tracked_inliers} {'<' if insert else '≥'} {min_inliers}",
    )


def decide_frame_distance(current_pose: np.ndarray, last_kf_pose: np.ndarray, *, min_distance_m: float = 0.5) -> KeyframeDecision:
    """``current_pose``, ``last_kf_pose``: row-major 4×4 SE(3) (T_cw)."""
    rel = current_pose @ np.linalg.inv(last_kf_pose)
    dist = se3_log_norm(rel)
    insert = dist >= min_distance_m
    return KeyframeDecision(
        insert=insert,
        policy=Policy.FRAME_DISTANCE,
        metric=float(dist),
        reason=f"|log(T_cur · T_last⁻¹)| = {dist:.3f} {'≥' if insert else '<'} {min_distance_m}",
    )


def decide_fixed_interval(frame_idx: int, last_kf_frame_idx: int, *, interval: int = 5) -> KeyframeDecision:
    delta = frame_idx - last_kf_frame_idx
    insert = delta >= interval
    return KeyframeDecision(
        insert=insert,
        policy=Policy.FIXED_INTERVAL,
        metric=float(delta),
        reason=f"frame Δ = {delta} {'≥' if insert else '<'} {interval}",
    )
