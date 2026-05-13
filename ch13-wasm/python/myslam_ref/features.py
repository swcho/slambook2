"""Feature detection + LK tracking — Python mirror of bind_features.cpp.

Option keys (``maxFeatures``, ``qualityLevel``, ``minDistance``, ``blockSize``,
``harrisK``, ``fastThreshold``, ``nonmaxSuppression``, ``orbScaleFactor``,
``orbNLevels``, ``mask``; LK: ``winSize``, ``maxLevel``, ``maxIter``, ``eps``,
``useInitialFlow``, ``initialPts``) are kept identical to the C++ binding so the
two sides can be diffed mechanically.
"""

from __future__ import annotations

from enum import IntEnum
from typing import Any

import cv2
import numpy as np


class Detector(IntEnum):
    GFTT = 0
    HARRIS = 1
    FAST = 2
    ORB = 3


def _resolve_mask(opts: dict[str, Any], w: int, h: int) -> np.ndarray | None:
    mask = opts.get("mask")
    if mask is None:
        return None
    arr = np.asarray(mask, dtype=np.uint8)
    if arr.size == w * h:
        arr = arr.reshape(h, w)
    if arr.shape != (h, w):
        return None
    return arr


def detect(img: np.ndarray, algo: Detector | int, **opts: Any) -> np.ndarray:
    """Run a detector and return an (N, 3) Float64 array of ``[x, y, score]``.

    Mirrors ``detectFeatures`` in bind_features.cpp.
    """
    if img.dtype != np.uint8 or img.ndim != 2:
        raise ValueError(f"detect expects grayscale uint8 (H, W); got dtype={img.dtype} shape={img.shape}")
    h, w = img.shape
    mask = _resolve_mask(opts, w, h)

    max_features = int(opts.get("maxFeatures", 150))
    quality_level = float(opts.get("qualityLevel", 0.01))
    min_distance = float(opts.get("minDistance", 20.0))
    block_size = int(opts.get("blockSize", 3))
    harris_k = float(opts.get("harrisK", 0.04))
    fast_threshold = int(opts.get("fastThreshold", 20))
    nonmax_suppression = bool(opts.get("nonmaxSuppression", True))
    orb_scale = float(opts.get("orbScaleFactor", 1.2))
    orb_levels = int(opts.get("orbNLevels", 8))

    algo_int = int(algo)
    if algo_int in (int(Detector.GFTT), int(Detector.HARRIS)):
        use_harris = algo_int == int(Detector.HARRIS)
        corners = cv2.goodFeaturesToTrack(
            img,
            maxCorners=max_features,
            qualityLevel=quality_level,
            minDistance=min_distance,
            mask=mask,
            blockSize=block_size,
            useHarrisDetector=use_harris,
            k=harris_k,
        )
        if corners is None:
            return np.zeros((0, 3), dtype=np.float64)
        pts = corners.reshape(-1, 2).astype(np.float64)
        scores = np.ones((pts.shape[0], 1), dtype=np.float64)
        return np.hstack([pts, scores])

    if algo_int == int(Detector.FAST):
        fast = cv2.FastFeatureDetector_create(threshold=fast_threshold, nonmaxSuppression=nonmax_suppression)
        kps = fast.detect(img, mask=mask)
        if not kps:
            return np.zeros((0, 3), dtype=np.float64)
        # Cap to max_features by descending response (matches C++ nth_element).
        if len(kps) > max_features:
            kps = sorted(kps, key=lambda kp: kp.response, reverse=True)[:max_features]
        return np.array([[kp.pt[0], kp.pt[1], kp.response] for kp in kps], dtype=np.float64)

    if algo_int == int(Detector.ORB):
        orb = cv2.ORB_create(nfeatures=max_features, scaleFactor=orb_scale, nlevels=orb_levels)
        kps = orb.detect(img, mask)
        if not kps:
            return np.zeros((0, 3), dtype=np.float64)
        return np.array([[kp.pt[0], kp.pt[1], kp.response] for kp in kps], dtype=np.float64)

    raise ValueError(f"unknown detector algo={algo_int}")


def track_lk(prev: np.ndarray, curr: np.ndarray, prev_pts: np.ndarray, **opts: Any) -> np.ndarray:
    """Pyramidal LK tracking. Returns (N, 3) of ``[x, y, status]`` matching trackLK in C++.

    ``prev_pts`` has shape (N, 3) where the score column is ignored on input.
    """
    if prev.shape != curr.shape:
        raise ValueError(f"prev/curr shape mismatch: {prev.shape} vs {curr.shape}")
    if prev.dtype != np.uint8 or curr.dtype != np.uint8:
        raise ValueError("LK expects uint8 grayscale frames")
    if prev_pts.ndim != 2 or prev_pts.shape[1] != 3:
        raise ValueError(f"prev_pts must be (N, 3); got {prev_pts.shape}")

    win = int(opts.get("winSize", 11))
    max_level = int(opts.get("maxLevel", 3))
    max_iter = int(opts.get("maxIter", 30))
    eps = float(opts.get("eps", 0.01))
    use_initial_flow = bool(opts.get("useInitialFlow", False))

    n = prev_pts.shape[0]
    p0 = prev_pts[:, :2].astype(np.float32).reshape(-1, 1, 2)

    p1_init: np.ndarray | None = None
    flags = 0
    if use_initial_flow:
        flags |= cv2.OPTFLOW_USE_INITIAL_FLOW
        init = opts.get("initialPts")
        if init is not None:
            init_arr = np.asarray(init, dtype=np.float32)
            if init_arr.ndim == 1 and init_arr.size % 3 == 0:
                init_arr = init_arr.reshape(-1, 3)
            if init_arr.ndim == 2 and init_arr.shape[1] == 3:
                cap = min(n, init_arr.shape[0])
                p1_init = p0.copy()
                p1_init[:cap, 0, 0] = init_arr[:cap, 0]
                p1_init[:cap, 0, 1] = init_arr[:cap, 1]
        if p1_init is None:
            # Fall back to p0 — same behavior as the C++ binding.
            p1_init = p0.copy()

    criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, max_iter, eps)
    p1, status, _err = cv2.calcOpticalFlowPyrLK(
        prev,
        curr,
        p0,
        p1_init,
        winSize=(win, win),
        maxLevel=max_level,
        criteria=criteria,
        flags=flags,
    )

    out = np.zeros((n, 3), dtype=np.float64)
    if p1 is None:
        return out
    p1_xy = p1.reshape(-1, 2)
    st = status.reshape(-1).astype(np.float64) if status is not None else np.zeros(n)
    out[:, 0] = p1_xy[:, 0]
    out[:, 1] = p1_xy[:, 1]
    out[:, 2] = st
    return out


# ---- Synthetic test frame (mirrors verify_features.ts:48-73) -----------------

_SYNTH_W = 1226
_SYNTH_H = 370


def render_synth_frame(frame_idx: int, camera_idx: int) -> np.ndarray:
    """Mirror of ``renderFrame`` in wasm-src/spike/verify_features.ts.

    Generates a deterministic 1226×370 grayscale image with two bright markers
    and a horizon stripe. Same x-shift convention so the LK disparity test
    (frame 0, cam 0 → cam 1 with stereoShift = 22 px) keeps producing the same
    expected dx ≈ −22.
    """
    forward_shift = frame_idx * 14
    stereo_shift = 0 if camera_idx == 0 else 22
    w, h = _SYNTH_W, _SYNTH_H
    img = np.zeros((h, w), dtype=np.uint8)

    # Vectorized version of the per-pixel loop in the TS implementation.
    x_grid, y_grid = np.meshgrid(np.arange(w), np.arange(h))
    sx = x_grid + stereo_shift + forward_shift
    v = 40 + ((sx >> 1) & 0x3F)
    v = v + np.where((sx + y_grid) % 48 < 2, 35, 0)
    v = v + np.where(((sx >> 5) ^ (y_grid >> 5)) & 1 == 1, 55, 0)
    v = np.where(y_grid == h // 2, 230, v)

    marker_x_left = int(w * 0.48) - forward_shift
    marker_x_right = marker_x_left - stereo_shift
    marker_y = int(h * 0.46)
    m2x_left = int(w * 0.25) - forward_shift
    m2x_right = m2x_left - 11
    m2y = int(h * 0.72)

    mx = marker_x_left if camera_idx == 0 else marker_x_right
    m2x = m2x_left if camera_idx == 0 else m2x_right

    in_marker1 = (np.abs(x_grid - mx) < 5) & (np.abs(y_grid - marker_y) < 5)
    in_marker2 = (np.abs(x_grid - m2x) < 4) & (np.abs(y_grid - m2y) < 4)
    v = np.where(in_marker1, 255, v)
    v = np.where(in_marker2, 250, v)

    np.clip(v, 0, 255, out=v)
    img[:] = v.astype(np.uint8)
    return img
