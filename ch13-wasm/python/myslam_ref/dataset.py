"""KITTI mini dataset loader — Python mirror of src/lib/kitti.ts.

Parses the odometry-style ``calib.txt`` (4 rows of ``P<k>: <12 floats>``) and
loads stereo PNG pairs from ``image_0/`` and ``image_1/``. Output shapes match
the TS loader so a Python reference can read the same fixtures the WASM build
ships with.

The TS loader applies a 0.5 downsample by default (UI display). For ground-truth
verification we default to ``downsample=1.0`` so K matches calib.txt verbatim.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

_DEFAULT_DATASET_DIR = Path(__file__).resolve().parents[2] / "public" / "datasets" / "kitti05-mini"


@dataclass(frozen=True)
class KittiCamera:
    """One rectified camera (matches the TS ``KittiCamera`` interface)."""

    id: int
    fx: float
    fy: float
    cx: float
    cy: float
    baseline_m: float  # |t|, t = K^{-1} * P[:, 3]
    t: tuple[float, float, float]
    K: np.ndarray  # (3, 3) float64 — already scaled by ``downsample``
    P: np.ndarray  # (3, 4) float64 — raw projection from calib.txt
    downsample: float


@dataclass
class StereoFrame:
    index: int
    left: np.ndarray  # (H, W) uint8
    right: np.ndarray

    @property
    def width(self) -> int:
        return self.left.shape[1]

    @property
    def height(self) -> int:
        return self.left.shape[0]


@dataclass
class KittiMini:
    cameras: list[KittiCamera]
    frames: list[StereoFrame]

    @property
    def camera0(self) -> KittiCamera:
        return self.cameras[0]

    @property
    def camera1(self) -> KittiCamera:
        return self.cameras[1]


def parse_kitti_calib(text: str, downsample: float = 1.0) -> list[KittiCamera]:
    """Parse odometry-style calib.txt — lines ``P<k>: <12 floats>``."""
    cameras: list[KittiCamera] = []
    pattern = re.compile(r"^P(\d):\s*(.+)$")
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        m = pattern.match(line)
        if not m:
            continue
        cam_id = int(m.group(1))
        nums = m.group(2).split()
        if len(nums) != 12:
            raise ValueError(f"calib.txt: P{cam_id} expects 12 floats, got {len(nums)}: {line}")
        p = np.asarray([float(x) for x in nums], dtype=np.float64)
        cameras.append(_build_camera(cam_id, p, downsample))
    if not cameras:
        raise ValueError("calib.txt contained no P<k> rows")
    cameras.sort(key=lambda c: c.id)
    return cameras


def _build_camera(cam_id: int, p: np.ndarray, downsample: float) -> KittiCamera:
    P = p.reshape(3, 4)
    raw_K = P[:, :3].copy()
    last_col = P[:, 3].copy()
    # t = K^{-1} * P[:, 3] — same convention as ch13's dataset.cpp and src/lib/kitti.ts.
    t_vec = np.linalg.solve(raw_K, last_col)
    baseline = float(np.linalg.norm(t_vec))

    # Apply downsample: scale the first two rows of K (rows for x and y); row 2 stays [0 0 1].
    K = raw_K.copy()
    K[0:2, :] *= downsample
    return KittiCamera(
        id=cam_id,
        fx=float(K[0, 0]),
        fy=float(K[1, 1]),
        cx=float(K[0, 2]),
        cy=float(K[1, 2]),
        baseline_m=baseline,
        t=(float(t_vec[0]), float(t_vec[1]), float(t_vec[2])),
        K=K,
        P=P,
        downsample=downsample,
    )


def load_kitti_mini(
    root: str | Path | None = None,
    *,
    downsample: float = 1.0,
    max_frames: int | None = None,
) -> KittiMini:
    """Load calib + stereo PNG pairs from a KITTI odometry-mini directory.

    Defaults to ``<repo>/ch13-wasm/public/datasets/kitti05-mini``. Returns a
    :class:`KittiMini` with all frames found under ``image_0/NNNNNN.png`` /
    ``image_1/NNNNNN.png``.
    """
    root_path = Path(root) if root is not None else _DEFAULT_DATASET_DIR
    if not root_path.exists():
        raise FileNotFoundError(f"KITTI mini root not found: {root_path}")

    calib_path = root_path / "calib.txt"
    if not calib_path.exists():
        raise FileNotFoundError(f"calib.txt not found at {calib_path}")
    cameras = parse_kitti_calib(calib_path.read_text(), downsample=downsample)

    left_dir = root_path / "image_0"
    right_dir = root_path / "image_1"
    frame_files = sorted(p.name for p in left_dir.glob("*.png"))
    if max_frames is not None:
        frame_files = frame_files[:max_frames]

    frames: list[StereoFrame] = []
    for fname in frame_files:
        idx = int(fname.split(".")[0])
        left = cv2.imread(str(left_dir / fname), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(right_dir / fname), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            raise FileNotFoundError(f"failed to decode frame {fname} (left={left is not None}, right={right is not None})")
        if left.shape != right.shape:
            raise ValueError(f"stereo size mismatch at {fname}: L={left.shape} R={right.shape}")
        if downsample != 1.0:
            new_w = max(1, round(left.shape[1] * downsample))
            new_h = max(1, round(left.shape[0] * downsample))
            left = cv2.resize(left, (new_w, new_h), interpolation=cv2.INTER_AREA)
            right = cv2.resize(right, (new_w, new_h), interpolation=cv2.INTER_AREA)
        frames.append(StereoFrame(index=idx, left=left, right=right))

    return KittiMini(cameras=cameras, frames=frames)
