"""Keyframe abstraction + on-disk format for SLAM/SfM experiments.

Storage layout (OpenMVG / Kapture style — JSON manifest + binary sidecars):

    <root>/
        keyframes.json                       # manifest: camera, detector meta, frames
        features/<detector>/<frame_id>.npz   # per-frame keypoints + descriptors

Camera intrinsics follow the COLMAP camera-model convention
(https://colmap.github.io/cameras.html), so the manifest interoperates with
COLMAP-style pipelines.

The on-disk keypoint array layout (N, 7) is:

    x, y, size, angle, response, octave, class_id

which is a superset of cv2.KeyPoint and matches what most SLAM/SfM pipelines
expose. Descriptor dtype/dim/norm are recorded per detector in the manifest.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

import cv2
import numpy as np

SCHEMA_VERSION = "1.0"
KEYPOINT_FIELDS = ("x", "y", "size", "angle", "response", "octave", "class_id")
KEYPOINT_NDIM = len(KEYPOINT_FIELDS)


# --------------------------------------------------------------------------- #
# Camera (COLMAP-compatible)
# --------------------------------------------------------------------------- #

# COLMAP camera models we support. Param order matches COLMAP exactly.
_CAMERA_PARAM_ORDER: dict[str, tuple[str, ...]] = {
    "SIMPLE_PINHOLE": ("f", "cx", "cy"),
    "PINHOLE": ("fx", "fy", "cx", "cy"),
    "SIMPLE_RADIAL": ("f", "cx", "cy", "k"),
    "RADIAL": ("f", "cx", "cy", "k1", "k2"),
    "OPENCV": ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2"),
}


@dataclass
class Camera:
    """COLMAP-compatible camera intrinsics."""

    model: str
    width: int
    height: int
    params: list[float]

    def __post_init__(self) -> None:
        if self.model not in _CAMERA_PARAM_ORDER:
            raise ValueError(f"unknown camera model: {self.model}")
        expected = len(_CAMERA_PARAM_ORDER[self.model])
        if len(self.params) != expected:
            raise ValueError(
                f"{self.model} expects {expected} params, got {len(self.params)}"
            )

    @classmethod
    def assumed_pinhole(cls, width: int, height: int) -> "Camera":
        """Fallback intrinsics when no calibration is available: fx=fy=W, principal point = center."""
        f = float(width)
        return cls("PINHOLE", width, height, [f, f, width / 2.0, height / 2.0])

    @property
    def K(self) -> np.ndarray:
        """3x3 intrinsic matrix (ignores distortion)."""
        names = _CAMERA_PARAM_ORDER[self.model]
        p = dict(zip(names, self.params))
        if "f" in p:
            fx = fy = p["f"]
        else:
            fx, fy = p["fx"], p["fy"]
        cx, cy = p["cx"], p["cy"]
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    def to_dict(self) -> dict:
        return {
            "model": self.model,
            "width": self.width,
            "height": self.height,
            "params": list(self.params),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Camera":
        return cls(d["model"], int(d["width"]), int(d["height"]), list(d["params"]))


# --------------------------------------------------------------------------- #
# Features
# --------------------------------------------------------------------------- #


@dataclass
class DetectorMeta:
    """Per-detector metadata stored in the manifest (shared across all frames)."""

    name: str
    descriptor_dim: int
    descriptor_dtype: str  # numpy dtype string, e.g. "float32" or "uint8"
    norm: str  # "L2" or "Hamming"

    def to_dict(self) -> dict:
        return {
            "descriptor_dim": self.descriptor_dim,
            "descriptor_dtype": self.descriptor_dtype,
            "norm": self.norm,
        }

    @classmethod
    def from_dict(cls, name: str, d: dict) -> "DetectorMeta":
        return cls(name, int(d["descriptor_dim"]), str(d["descriptor_dtype"]), str(d["norm"]))


@dataclass
class FeatureSet:
    """Keypoints + descriptors from one detector applied to one frame."""

    detector: str
    keypoints: np.ndarray   # (N, 7) float32, columns = KEYPOINT_FIELDS
    descriptors: np.ndarray  # (N, D)

    def __post_init__(self) -> None:
        if self.keypoints.ndim != 2 or self.keypoints.shape[1] != KEYPOINT_NDIM:
            raise ValueError(
                f"keypoints must be (N, {KEYPOINT_NDIM}); got {self.keypoints.shape}"
            )
        if len(self.keypoints) != len(self.descriptors):
            raise ValueError(
                f"keypoint/descriptor count mismatch: "
                f"{len(self.keypoints)} vs {len(self.descriptors)}"
            )

    def __len__(self) -> int:
        return len(self.keypoints)

    @classmethod
    def from_cv(
        cls, detector: str, kps: list[cv2.KeyPoint], desc: np.ndarray | None,
        descriptor_dim: int, descriptor_dtype: str,
    ) -> "FeatureSet":
        kp_arr = np.array(
            [(k.pt[0], k.pt[1], k.size, k.angle, k.response, k.octave, k.class_id)
             for k in kps],
            dtype=np.float32,
        ).reshape(-1, KEYPOINT_NDIM)
        if desc is None or len(desc) == 0:
            desc = np.zeros((0, descriptor_dim), dtype=np.dtype(descriptor_dtype))
        return cls(detector, kp_arr, desc.astype(np.dtype(descriptor_dtype), copy=False))

    def to_cv_keypoints(self) -> list[cv2.KeyPoint]:
        return [
            cv2.KeyPoint(
                x=float(r[0]), y=float(r[1]),
                size=float(r[2]), angle=float(r[3]),
                response=float(r[4]), octave=int(r[5]), class_id=int(r[6]),
            )
            for r in self.keypoints
        ]

    @property
    def xy(self) -> np.ndarray:
        """(N, 2) keypoint coordinates."""
        return self.keypoints[:, :2]


# --------------------------------------------------------------------------- #
# Keyframe & Store
# --------------------------------------------------------------------------- #


@dataclass
class Keyframe:
    """One image + features from one or more detectors. Pose is optional."""

    id: str
    image_path: str  # relative to store root
    timestamp: float | None = None
    features: dict[str, FeatureSet] = field(default_factory=dict)
    # Optional pose, world-from-camera, as a 4x4 row-major matrix.
    pose_wc: np.ndarray | None = None

    def __getitem__(self, detector: str) -> FeatureSet:
        return self.features[detector]


@dataclass
class KeyframeStore:
    """Manages a manifest + sidecar directory on disk."""

    root: Path
    dataset: str
    camera: Camera
    detectors: dict[str, DetectorMeta] = field(default_factory=dict)
    frames: list[Keyframe] = field(default_factory=list)

    MANIFEST_NAME = "keyframes.json"

    # ---- mutation ----------------------------------------------------------

    def register_detector(self, meta: DetectorMeta) -> None:
        existing = self.detectors.get(meta.name)
        if existing and existing != meta:
            raise ValueError(
                f"detector {meta.name!r} already registered with different metadata"
            )
        self.detectors[meta.name] = meta

    def add_frame(self, kf: Keyframe) -> None:
        self.frames.append(kf)

    # ---- save --------------------------------------------------------------

    def _sidecar_path(self, frame_id: str, detector: str) -> Path:
        return self.root / "features" / detector / f"{frame_id}.npz"

    def save(self) -> Path:
        """Write manifest + per-frame npz sidecars. Returns manifest path."""
        self.root.mkdir(parents=True, exist_ok=True)

        manifest_frames = []
        for kf in self.frames:
            feat_entries: dict[str, dict] = {}
            for det_name, fs in kf.features.items():
                if det_name not in self.detectors:
                    raise ValueError(
                        f"frame {kf.id!r} has features for unregistered detector {det_name!r}"
                    )
                sidecar = self._sidecar_path(kf.id, det_name)
                sidecar.parent.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(
                    sidecar, keypoints=fs.keypoints, descriptors=fs.descriptors,
                )
                feat_entries[det_name] = {
                    "count": int(len(fs)),
                    "sidecar": str(sidecar.relative_to(self.root)),
                }

            entry: dict = {
                "id": kf.id,
                "image_path": kf.image_path,
                "features": feat_entries,
            }
            if kf.timestamp is not None:
                entry["timestamp"] = float(kf.timestamp)
            if kf.pose_wc is not None:
                entry["pose_wc"] = kf.pose_wc.tolist()
            manifest_frames.append(entry)

        manifest = {
            "version": SCHEMA_VERSION,
            "dataset": self.dataset,
            "camera": self.camera.to_dict(),
            "detectors": {n: m.to_dict() for n, m in self.detectors.items()},
            "frames": manifest_frames,
        }
        manifest_path = self.root / self.MANIFEST_NAME
        with manifest_path.open("w") as f:
            json.dump(manifest, f, indent=2)
        return manifest_path

    # ---- load --------------------------------------------------------------

    @classmethod
    def load(cls, root: Path, *, load_features: bool = True) -> "KeyframeStore":
        root = Path(root)
        with (root / cls.MANIFEST_NAME).open() as f:
            m = json.load(f)
        if m.get("version") != SCHEMA_VERSION:
            raise ValueError(f"unsupported schema version: {m.get('version')!r}")

        store = cls(
            root=root,
            dataset=m["dataset"],
            camera=Camera.from_dict(m["camera"]),
            detectors={n: DetectorMeta.from_dict(n, d) for n, d in m["detectors"].items()},
        )

        for fr in m["frames"]:
            kf = Keyframe(
                id=fr["id"],
                image_path=fr["image_path"],
                timestamp=fr.get("timestamp"),
            )
            if "pose_wc" in fr:
                kf.pose_wc = np.array(fr["pose_wc"], dtype=np.float64)
            if load_features:
                for det_name, entry in fr["features"].items():
                    with np.load(root / entry["sidecar"]) as z:
                        kf.features[det_name] = FeatureSet(
                            detector=det_name,
                            keypoints=z["keypoints"].astype(np.float32),
                            descriptors=z["descriptors"],
                        )
            store.frames.append(kf)
        return store

    # ---- convenience -------------------------------------------------------

    def frame_by_id(self, frame_id: str) -> Keyframe:
        for kf in self.frames:
            if kf.id == frame_id:
                return kf
        raise KeyError(frame_id)


# --------------------------------------------------------------------------- #
# Detector adapters (OpenCV-backed)
# --------------------------------------------------------------------------- #


class Detector(Protocol):
    """A detector knows its descriptor format and can run on a grayscale image."""

    meta: DetectorMeta

    def detect_and_compute(self, gray: np.ndarray) -> FeatureSet: ...


@dataclass
class _CvDetector:
    """Shared implementation for OpenCV (Feature2D)-style detectors."""

    meta: DetectorMeta
    _cv: cv2.Feature2D

    def detect_and_compute(self, gray: np.ndarray) -> FeatureSet:
        kps, desc = self._cv.detectAndCompute(gray, None)
        return FeatureSet.from_cv(
            self.meta.name, list(kps), desc,
            self.meta.descriptor_dim, self.meta.descriptor_dtype,
        )


def make_sift(n_features: int = 0) -> _CvDetector:
    """SIFT — Lowe 2004. 128-d float, L2 norm. de facto SfM default."""
    return _CvDetector(
        DetectorMeta("sift", 128, "float32", "L2"),
        cv2.SIFT_create(nfeatures=n_features),
    )


def make_orb(n_features: int = 2000) -> _CvDetector:
    """ORB — Rublee 2011. 32-byte (256-bit) Hamming. Used by ORB-SLAM."""
    return _CvDetector(
        DetectorMeta("orb", 32, "uint8", "Hamming"),
        cv2.ORB_create(nfeatures=n_features),
    )


def make_akaze() -> _CvDetector:
    """AKAZE — Alcantarilla 2013. Default MLDB descriptor: 61-byte Hamming."""
    return _CvDetector(
        DetectorMeta("akaze", 61, "uint8", "Hamming"),
        cv2.AKAZE_create(),
    )


def make_brisk() -> _CvDetector:
    """BRISK — Leutenegger 2011. 64-byte Hamming."""
    return _CvDetector(
        DetectorMeta("brisk", 64, "uint8", "Hamming"),
        cv2.BRISK_create(),
    )


DETECTOR_FACTORIES = {
    "sift": make_sift,
    "orb": make_orb,
    "akaze": make_akaze,
    "brisk": make_brisk,
}


def matcher_for(detector_meta: DetectorMeta, *, cross_check: bool = False) -> cv2.BFMatcher:
    """Return a BFMatcher with the correct norm for the given detector."""
    norm = cv2.NORM_L2 if detector_meta.norm == "L2" else cv2.NORM_HAMMING
    return cv2.BFMatcher(norm, crossCheck=cross_check)
