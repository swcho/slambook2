"""Relative pose estimation + monocular trajectory accumulation.

Conventions
-----------
- All poses are 4x4 `T_wc` (world-from-camera): columns of the upper-left 3x3
  are camera axes expressed in world frame; the last column is camera position
  in world frame.
- `recoverPose(E, kp_a, kp_b, K)` returns `R, t` such that `x_b = R·x_a + t`,
  i.e. `T_{b←a}`. To chain world poses we therefore use
  `T_wc[i+1] = T_wc[i] · inv(T_{b←a}) = T_wc[i] · [R^T | -R^T·t]`.
- Monocular: `‖t‖ = 1` per pair (scale ambiguity). Trajectory shape/direction
  are meaningful; absolute distance is not. This module does **not** triangulate
  or rescale across pairs — every pair contributes one unit-length step.
- Failed pairs split the trajectory into independent segments. Across segments
  the world frames are unrelated; each segment starts at identity.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from keyframe import Camera

SCHEMA_VERSION = "1.0"


# --------------------------------------------------------------------------- #
# Relative pose between two frames
# --------------------------------------------------------------------------- #


@dataclass
class RelativePose:
    """Result of a single (frame_a, frame_b) pose estimation."""

    frame_a: int
    frame_b: int
    success: bool
    method: str               # e.g. "essential_ransac" / "homography"
    n_matches: int            # input correspondences
    n_inliers_model: int      # inliers to E or H
    n_inliers_pose: int       # inliers passing cheirality (essential only)
    R: np.ndarray = field(default_factory=lambda: np.eye(3))   # (3,3) T_{b<-a}
    t: np.ndarray = field(default_factory=lambda: np.zeros(3))  # (3,) unit-norm

    def T_ba(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3] = self.t
        return T

    def T_ab(self) -> np.ndarray:
        """Inverse — used to chain world-from-camera poses."""
        T = np.eye(4)
        T[:3, :3] = self.R.T
        T[:3, 3] = -self.R.T @ self.t
        return T


# --------------------------------------------------------------------------- #
# Estimators
# --------------------------------------------------------------------------- #


@dataclass
class EssentialMatrixEstimator:
    """5-point algorithm + RANSAC, followed by cheirality-checked recoverPose."""

    threshold_px: float = 1.0
    prob: float = 0.999
    min_matches: int = 8         # 5 is theoretical minimum; >=8 for stability
    min_inliers_pose: int = 15   # below this, treat the pair as failed

    method = "essential_ransac"

    def estimate(
        self, pts_a: np.ndarray, pts_b: np.ndarray, K: np.ndarray,
        *, frame_a: int, frame_b: int,
    ) -> RelativePose:
        n = len(pts_a)
        out = RelativePose(
            frame_a=frame_a, frame_b=frame_b,
            success=False, method=self.method,
            n_matches=n, n_inliers_model=0, n_inliers_pose=0,
        )
        if n < self.min_matches:
            return out

        E, mask_e = cv2.findEssentialMat(
            pts_a, pts_b, K,
            method=cv2.RANSAC, prob=self.prob, threshold=self.threshold_px,
        )
        if E is None or E.shape != (3, 3):
            return out
        out.n_inliers_model = int(mask_e.sum())

        n_pose, R, t, _ = cv2.recoverPose(E, pts_a, pts_b, K, mask=mask_e)
        out.n_inliers_pose = int(n_pose)
        if n_pose < self.min_inliers_pose:
            return out

        out.R = R.astype(np.float64)
        out.t = t.ravel().astype(np.float64)
        out.success = True
        return out


@dataclass
class HomographyEstimator:
    """Planar-scene fallback. `decomposeHomographyMat` returns up to 4 candidate
    (R, t, n) solutions; we pick the one with the most points in front of both
    cameras (cheirality)."""

    threshold_px: float = 3.0
    min_matches: int = 8

    method = "homography"

    def estimate(
        self, pts_a: np.ndarray, pts_b: np.ndarray, K: np.ndarray,
        *, frame_a: int, frame_b: int,
    ) -> RelativePose:
        n = len(pts_a)
        out = RelativePose(
            frame_a=frame_a, frame_b=frame_b,
            success=False, method=self.method,
            n_matches=n, n_inliers_model=0, n_inliers_pose=0,
        )
        if n < self.min_matches:
            return out

        H, mask = cv2.findHomography(
            pts_a, pts_b, method=cv2.RANSAC, ransacReprojThreshold=self.threshold_px,
        )
        if H is None:
            return out
        out.n_inliers_model = int(mask.sum())

        ret, Rs, ts, _ns = cv2.decomposeHomographyMat(H, K)
        if ret == 0:
            return out

        # Cheirality: pick (R, t) maximizing # inliers in front of both cameras.
        inliers = mask.ravel().astype(bool)
        ia = _to_normalized(pts_a[inliers], K)
        ib = _to_normalized(pts_b[inliers], K)

        best_count, best_R, best_t = -1, Rs[0], ts[0]
        for R, t in zip(Rs, ts):
            count = _cheirality_count(R, t.ravel(), ia, ib)
            if count > best_count:
                best_count, best_R, best_t = count, R, t.ravel()

        out.R = best_R.astype(np.float64)
        out.t = (best_t / max(np.linalg.norm(best_t), 1e-12)).astype(np.float64)
        out.n_inliers_pose = int(best_count)
        out.success = best_count >= 8
        return out


def _to_normalized(pts: np.ndarray, K: np.ndarray) -> np.ndarray:
    """(N, 2) pixels -> (N, 2) normalized image coords."""
    Kinv = np.linalg.inv(K)
    h = np.concatenate([pts.astype(np.float64), np.ones((len(pts), 1))], axis=1)
    n = h @ Kinv.T
    return n[:, :2]


def _cheirality_count(R: np.ndarray, t: np.ndarray,
                      ia: np.ndarray, ib: np.ndarray) -> int:
    """How many triangulated points lie in front of both cameras."""
    if len(ia) == 0:
        return 0
    P1 = np.hstack([np.eye(3), np.zeros((3, 1))])
    P2 = np.hstack([R, t.reshape(3, 1)])
    pts4d = cv2.triangulatePoints(P1, P2, ia.T, ib.T)
    X1 = pts4d[:3] / np.where(pts4d[3:4] != 0, pts4d[3:4], 1e-12)
    X2 = R @ X1 + t.reshape(3, 1)
    return int(np.sum((X1[2] > 0) & (X2[2] > 0)))


# --------------------------------------------------------------------------- #
# Trajectory
# --------------------------------------------------------------------------- #


@dataclass
class Trajectory:
    """Per-frame world-from-camera poses (with NaN for invalid frames),
    grouped into continuous-valid `segments`."""

    detector: str
    method: str
    camera: Camera
    frame_ids: list[str]
    poses_wc: np.ndarray          # (N, 4, 4) — NaN where invalid
    valid: np.ndarray             # (N,) bool
    segments: list[tuple[int, int]]   # half-open [start, end)
    pairs: list[RelativePose]
    extra: dict = field(default_factory=dict)

    # ---- accumulation ------------------------------------------------------

    @classmethod
    def from_pairs(
        cls, detector: str, method: str, camera: Camera,
        frame_ids: list[str], pairs: list[RelativePose],
    ) -> "Trajectory":
        n = len(frame_ids)
        assert len(pairs) == n - 1, f"need {n - 1} pairs for {n} frames, got {len(pairs)}"

        poses = np.full((n, 4, 4), np.nan, dtype=np.float64)
        valid = np.zeros(n, dtype=bool)
        segments: list[tuple[int, int]] = []

        i = 0
        while i < n:
            # advance i to a new segment start
            seg_start = i
            poses[i] = np.eye(4)
            valid[i] = True
            j = i
            while j < n - 1 and pairs[j].success:
                poses[j + 1] = poses[j] @ pairs[j].T_ab()
                valid[j + 1] = True
                j += 1
            segments.append((seg_start, j + 1))
            # next segment begins after the failed pair (skip frame j+1 entirely)
            if j == n - 1:
                break
            i = j + 2  # frame j+1 had no incoming valid pose; restart at j+2
        # final orphan frame (no leading pair) becomes a 1-frame segment at identity
        # — already handled because we seed each segment with identity.

        return cls(
            detector=detector,
            method=method,
            camera=camera,
            frame_ids=list(frame_ids),
            poses_wc=poses,
            valid=valid,
            segments=segments,
            pairs=list(pairs),
        )

    # ---- accessors ---------------------------------------------------------

    @property
    def positions(self) -> np.ndarray:
        """(N, 3) camera positions in world frame; NaN where invalid."""
        return self.poses_wc[:, :3, 3]

    def segment_positions(self) -> list[np.ndarray]:
        return [self.positions[s:e] for s, e in self.segments if e - s >= 2]

    def summary(self) -> dict:
        n = len(self.frame_ids)
        return {
            "n_frames": n,
            "n_valid": int(self.valid.sum()),
            "n_segments": len(self.segments),
            "longest_segment": max((e - s for s, e in self.segments), default=0),
            "n_success_pairs": int(sum(p.success for p in self.pairs)),
            "n_total_pairs": len(self.pairs),
        }

    # ---- save / load -------------------------------------------------------

    def to_json(self) -> dict:
        return {
            "version": SCHEMA_VERSION,
            "detector": self.detector,
            "method": self.method,
            "camera": self.camera.to_dict(),
            "frame_ids": self.frame_ids,
            "valid": self.valid.tolist(),
            "segments": [list(s) for s in self.segments],
            "poses_wc": [
                p.tolist() if not np.isnan(p).any() else None
                for p in self.poses_wc
            ],
            "pairs": [
                {
                    "frame_a": p.frame_a, "frame_b": p.frame_b,
                    "success": bool(p.success),
                    "method": p.method,
                    "n_matches": int(p.n_matches),
                    "n_inliers_model": int(p.n_inliers_model),
                    "n_inliers_pose": int(p.n_inliers_pose),
                    "R": p.R.tolist() if p.success else None,
                    "t": p.t.tolist() if p.success else None,
                }
                for p in self.pairs
            ],
            "extra": self.extra,
        }

    def save(self, path: Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w") as f:
            json.dump(self.to_json(), f, indent=2)
        return path

    @classmethod
    def load(cls, path: Path) -> "Trajectory":
        with Path(path).open() as f:
            d = json.load(f)
        if d.get("version") != SCHEMA_VERSION:
            raise ValueError(f"unsupported trajectory schema: {d.get('version')}")

        n = len(d["frame_ids"])
        poses = np.full((n, 4, 4), np.nan, dtype=np.float64)
        for i, p in enumerate(d["poses_wc"]):
            if p is not None:
                poses[i] = np.array(p, dtype=np.float64)

        pairs: list[RelativePose] = []
        for pd in d["pairs"]:
            rp = RelativePose(
                frame_a=pd["frame_a"], frame_b=pd["frame_b"],
                success=pd["success"], method=pd["method"],
                n_matches=pd["n_matches"],
                n_inliers_model=pd["n_inliers_model"],
                n_inliers_pose=pd["n_inliers_pose"],
            )
            if pd["success"]:
                rp.R = np.array(pd["R"], dtype=np.float64)
                rp.t = np.array(pd["t"], dtype=np.float64)
            pairs.append(rp)

        return cls(
            detector=d["detector"],
            method=d["method"],
            camera=Camera.from_dict(d["camera"]),
            frame_ids=list(d["frame_ids"]),
            poses_wc=poses,
            valid=np.array(d["valid"], dtype=bool),
            segments=[tuple(s) for s in d["segments"]],
            pairs=pairs,
            extra=d.get("extra", {}),
        )
