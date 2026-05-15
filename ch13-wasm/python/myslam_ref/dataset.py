"""KITTI mini dataset loader — Python mirror of src/lib/kitti.ts.

Supports two KITTI calibration formats (same as the TS loader):

* odometry ``calib.txt``           — rows ``P<k>: <12 floats>``
* raw      ``calib_cam_to_cam.txt`` — rows ``P_rect_0<k>: <12 floats>``

Stereo PNG pairs are read from ``image_0/`` and ``image_1/``. Output shapes
match the TS loader so a Python reference can read the same fixtures the WASM
build ships with.

The TS loader applies a 0.5 downsample by default (UI display). For
ground-truth verification we default to ``downsample=1.0`` so K matches the
calibration file verbatim.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

_DEFAULT_DATASET_DIR = Path(
    __file__).resolve().parents[2] / "public" / "datasets"
_DEFAULT_DATASET_DIR_MINI = _DEFAULT_DATASET_DIR / "kitti05-mini"


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


_ODOMETRY_ROW = re.compile(r"^P(\d):\s*(.+)$")
_RAW_ROW = re.compile(r"^P_rect_0(\d):\s*(.+)$")


def parse_kitti_calib(text: str, downsample: float = 1.0) -> list[KittiCamera]:
    """Parse odometry-style calib.txt — lines ``P<k>: <12 floats>``."""
    return _parse_projection_rows(
        text, _ODOMETRY_ROW, downsample, label="calib.txt", row_name="P"
    )


def parse_kitti_raw_calib(text: str, downsample: float = 1.0) -> list[KittiCamera]:
    """Parse raw-style calib_cam_to_cam.txt — lines ``P_rect_0<k>: <12 floats>``.

    Other rows (``S_``, ``K_``, ``D_``, ``R_``, ``T_``, ``R_rect_``) are
    ignored — we only need the post-rectification projection matrices that
    apply to the ``image_0X`` streams.
    """
    return _parse_projection_rows(
        text,
        _RAW_ROW,
        downsample,
        label="calib_cam_to_cam.txt",
        row_name="P_rect_0",
    )


def parse_kitti_calib_auto(text: str, downsample: float = 1.0) -> list[KittiCamera]:
    """Parse either calib format, dispatching on whichever row pattern matches."""
    if _RAW_ROW.search(text) is not None or any(
        _RAW_ROW.match(line.strip()) for line in text.splitlines()
    ):
        return parse_kitti_raw_calib(text, downsample=downsample)
    return parse_kitti_calib(text, downsample=downsample)


def _parse_projection_rows(
    text: str,
    pattern: re.Pattern[str],
    downsample: float,
    *,
    label: str,
    row_name: str,
) -> list[KittiCamera]:
    cameras: list[KittiCamera] = []
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
            raise ValueError(
                f"{label}: {row_name}{cam_id} expects 12 floats, got {len(nums)}: {line}"
            )
        p = np.asarray([float(x) for x in nums], dtype=np.float64)
        cameras.append(_build_camera(cam_id, p, downsample))
    if not cameras:
        raise ValueError(f"{label} contained no {row_name}<k> rows")
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


def _resolve_calib_path(root_path: Path) -> Path:
    """Pick the calibration file, preferring the raw filename when present."""
    for name in ("calib_cam_to_cam.txt", "calib.txt"):
        candidate = root_path / name
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        f"no calib_cam_to_cam.txt or calib.txt under {root_path}"
    )


def load_kitti_dataset(
    dataset: str | Path | None = None,
    *,
    downsample: float = 1.0,
    max_frames: int | None = None,
) -> KittiMini:
    """Load calib + stereo PNG pairs from a KITTI mini directory.

    Defaults to ``<repo>/ch13-wasm/public/datasets/kitti05-mini``. Accepts both
    odometry (``calib.txt``: ``P<k>`` rows) and raw (``calib_cam_to_cam.txt``:
    ``P_rect_0<k>`` rows) layouts; the calibration file is picked by name and
    parsed by content. Returns a :class:`KittiMini` with all frames found
    under ``image_0/NNNNNN.png`` / ``image_1/NNNNNN.png``.
    """
    root_path = Path(_DEFAULT_DATASET_DIR /
                     dataset) if dataset is not None else _DEFAULT_DATASET_DIR_MINI
    if not root_path.exists():
        raise FileNotFoundError(f"KITTI mini root not found: {root_path}")

    calib_path = _resolve_calib_path(root_path)
    cameras = parse_kitti_calib_auto(calib_path.read_text(), downsample=downsample)

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
            raise FileNotFoundError(
                f"failed to decode frame {fname} (left={left is not None}, right={right is not None})")
        if left.shape != right.shape:
            raise ValueError(
                f"stereo size mismatch at {fname}: L={left.shape} R={right.shape}")
        if downsample != 1.0:
            new_w = max(1, round(left.shape[1] * downsample))
            new_h = max(1, round(left.shape[0] * downsample))
            left = cv2.resize(left, (new_w, new_h),
                              interpolation=cv2.INTER_AREA)
            right = cv2.resize(right, (new_w, new_h),
                               interpolation=cv2.INTER_AREA)
        frames.append(StereoFrame(index=idx, left=left, right=right))

    return KittiMini(cameras=cameras, frames=frames)
