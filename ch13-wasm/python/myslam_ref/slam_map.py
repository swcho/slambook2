"""Sliding-window SlamMap — Python mirror of src/lib/slam/map.ts.

Four eviction policies (``ch13-default`` / ``fifo`` / ``covisibility`` /
``distance-only``) + ``clean_map`` that drops orphan landmarks, matching the
behavior verified by ``wasm-src/spike/verify_slam.ts``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from .se3 import se3_invert, se3_log_norm, se3_translation


class Policy(str, Enum):
    CH13_DEFAULT = "ch13-default"
    FIFO = "fifo"
    COVISIBILITY = "covisibility"
    DISTANCE_ONLY = "distance-only"


@dataclass
class KeyFrame:
    id: int
    frame_id: int
    insert_order: int
    pose: np.ndarray  # (4, 4) row-major SE(3)
    observed_landmark_ids: set[int] = field(default_factory=set)


@dataclass
class MapPoint:
    id: int
    pos: np.ndarray  # (3,)
    observing_kf_ids: set[int] = field(default_factory=set)


@dataclass
class EvictionRecord:
    evicted_kf_id: int
    reason: str
    metric: float
    landmarks_removed: int


@dataclass
class PolicyOptions:
    duplicate_distance_threshold: float = 0.2


class SlamMap:
    def __init__(self, num_active_keyframes: int = 7):
        self.keyframes: dict[int, KeyFrame] = {}
        self.landmarks: dict[int, MapPoint] = {}
        self.active_keyframe_ids: set[int] = set()
        self.active_landmark_ids: set[int] = set()
        self.num_active_keyframes = num_active_keyframes
        self._next_kf_id = 0
        self._next_mp_id = 0
        self._next_insert_order = 0

    def insert_map_point(self, pos: np.ndarray | list[float]) -> int:
        mp_id = self._next_mp_id
        self._next_mp_id += 1
        self.landmarks[mp_id] = MapPoint(id=mp_id, pos=np.asarray(pos, dtype=np.float64))
        self.active_landmark_ids.add(mp_id)
        return mp_id

    def insert_keyframe(
        self,
        frame_id: int,
        pose: np.ndarray,
        observed_landmark_ids: set[int],
        policy: Policy | str,
        opts: PolicyOptions | None = None,
    ) -> tuple[KeyFrame, EvictionRecord | None]:
        opts = opts or PolicyOptions()
        kf = KeyFrame(
            id=self._next_kf_id,
            frame_id=frame_id,
            insert_order=self._next_insert_order,
            pose=np.asarray(pose, dtype=np.float64).reshape(4, 4),
            observed_landmark_ids=set(observed_landmark_ids),
        )
        self._next_kf_id += 1
        self._next_insert_order += 1
        self.keyframes[kf.id] = kf
        self.active_keyframe_ids.add(kf.id)
        for lm_id in observed_landmark_ids:
            lm = self.landmarks.get(lm_id)
            if lm is None:
                continue
            lm.observing_kf_ids.add(kf.id)
            self.active_landmark_ids.add(lm_id)
        eviction = None
        if len(self.active_keyframe_ids) > self.num_active_keyframes:
            eviction = self._remove_old_keyframe(kf, Policy(policy), opts)
        return kf, eviction

    def _get_active_keyframes(self) -> list[KeyFrame]:
        out = [self.keyframes[i] for i in self.active_keyframe_ids if i in self.keyframes]
        out.sort(key=lambda k: k.insert_order)
        return out

    def _remove_old_keyframe(
        self,
        current_kf: KeyFrame,
        policy: Policy,
        opts: PolicyOptions,
    ) -> EvictionRecord | None:
        candidates = [kf for kf in self._get_active_keyframes() if kf.id != current_kf.id]
        if not candidates:
            return None

        twc = se3_invert(current_kf.pose)
        distances = {kf.id: se3_log_norm(kf.pose @ twc) for kf in candidates}

        evicted_id = -1
        reason = ""
        metric = 0.0

        if policy == Policy.CH13_DEFAULT:
            min_id, max_id = -1, -1
            min_dis, max_dis = float("inf"), float("-inf")
            for kf in candidates:
                d = distances[kf.id]
                if d < min_dis:
                    min_dis, min_id = d, kf.id
                if d > max_dis:
                    max_dis, max_id = d, kf.id
            if min_dis < opts.duplicate_distance_threshold:
                evicted_id, metric = min_id, min_dis
                reason = f"closest (d={min_dis:.3f} < {opts.duplicate_distance_threshold}) — ch13 duplicate branch"
            else:
                evicted_id, metric = max_id, max_dis
                reason = f"farthest (no duplicates within {opts.duplicate_distance_threshold} m) — ch13 diversity branch"
        elif policy == Policy.FIFO:
            oldest = float("inf")
            for kf in candidates:
                if kf.insert_order < oldest:
                    oldest, evicted_id = kf.insert_order, kf.id
            metric = oldest
            reason = f"oldest insertion order {oldest}"
        elif policy == Policy.COVISIBILITY:
            min_shared = float("inf")
            for kf in candidates:
                shared = len(kf.observed_landmark_ids & current_kf.observed_landmark_ids)
                if shared < min_shared:
                    min_shared, evicted_id = shared, kf.id
            metric = min_shared
            reason = f"fewest shared landmarks ({min_shared}) with current KF"
        elif policy == Policy.DISTANCE_ONLY:
            min_dis = float("inf")
            for kf in candidates:
                d = distances[kf.id]
                if d < min_dis:
                    min_dis, evicted_id = d, kf.id
            metric = min_dis
            reason = f"closest only (d={min_dis:.3f})"

        if evicted_id < 0:
            return None
        evicted_kf = self.keyframes.get(evicted_id)
        if evicted_kf is None:
            return None
        self.active_keyframe_ids.discard(evicted_id)
        for lm_id in evicted_kf.observed_landmark_ids:
            lm = self.landmarks.get(lm_id)
            if lm:
                lm.observing_kf_ids.discard(evicted_id)
        removed = self.clean_map()
        return EvictionRecord(evicted_kf_id=evicted_id, reason=reason, metric=float(metric), landmarks_removed=removed)

    def clean_map(self) -> int:
        removed = 0
        for lm_id in list(self.active_landmark_ids):
            lm = self.landmarks.get(lm_id)
            if lm is None:
                continue
            still_active = any(kf_id in self.active_keyframe_ids for kf_id in lm.observing_kf_ids)
            if not still_active:
                self.active_landmark_ids.discard(lm_id)
                removed += 1
        return removed

    def active_pose_spread(self) -> tuple[np.ndarray, float, int]:
        ts = [se3_translation(se3_invert(kf.pose)) for kf in self._get_active_keyframes()]
        if not ts:
            return np.zeros(3), 0.0, 0
        ts_arr = np.asarray(ts)
        mean_t = ts_arr.mean(axis=0)
        variance = float(((ts_arr - mean_t) ** 2).sum(axis=1).mean())
        return mean_t, variance, ts_arr.shape[0]


# ---- Synthetic KF stream — mirrors src/lib/slam/map.ts:buildSyntheticKfStream


def _lcg_rand():
    state = [0xC13D]

    def rand() -> float:
        state[0] = (state[0] * 1103515245 + 12345) & 0x7FFFFFFF
        return state[0] / 0x7FFFFFFF

    return rand


def build_synthetic_kf_stream(num_forward_a: int = 5, num_duplicates: int = 2, num_forward_b: int = 3):
    rand = _lcg_rand()
    num_landmarks = 80
    landmark_positions = np.array([
        [(rand() - 0.5) * 6, (rand() - 0.5) * 4, 4 + rand() * 12]
        for _ in range(num_landmarks)
    ])

    poses: list[np.ndarray] = []
    for i in range(num_forward_a):
        tz = -i * 0.6
        p = np.eye(4)
        p[2, 3] = tz
        poses.append(p)
    base_tz = -(num_forward_a - 1) * 0.6
    for i in range(num_duplicates):
        dx = 0.05 * (i + 1)
        dz = 0.05 * (i + 1)
        p = np.eye(4)
        p[0, 3] = dx
        p[2, 3] = base_tz + dz
        poses.append(p)
    for i in range(1, num_forward_b + 1):
        tz = base_tz - i * 1.0
        p = np.eye(4)
        p[2, 3] = tz
        poses.append(p)

    observed_landmark_ids: list[set[int]] = []
    for pose in poses:
        twc = se3_invert(pose)
        cx, cz = twc[0, 3], twc[2, 3]
        dists: list[tuple[int, float]] = []
        for i, lp in enumerate(landmark_positions):
            cam_to_lm = lp[2] - cz
            if cam_to_lm < 1.0 or cam_to_lm > 20:
                continue
            dx = lp[0] - cx
            dy = lp[1]
            dists.append((i, float(np.hypot(dx, dy))))
        dists.sort(key=lambda x: x[1])
        observed_landmark_ids.append({i for i, _ in dists[:20]})

    return {
        "poses": poses,
        "observed_landmark_ids": observed_landmark_ids,
        "num_landmarks": num_landmarks,
        "landmark_positions": landmark_positions,
    }
