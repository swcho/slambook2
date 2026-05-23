# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Keyframe 데이터 생성 — SIFT / ORB / AKAZE / BRISK
#
# `data/260523_house/images/frame_*.jpg` 의 모든 프레임에 대해
# 여러 detector를 돌려 `KeyframeStore` 포맷으로 저장한다.
#
# 출력
# - `data/260523_house/keyframes/keyframes.json`  — manifest
# - `data/260523_house/keyframes/features/<detector>/frame_*.npz` — 사이드카

# %%
import time
from pathlib import Path

import cv2

from keyframe import (
    DETECTOR_FACTORIES,
    Camera,
    Keyframe,
    KeyframeStore,
)

HERE = Path.cwd() if "__file__" not in globals() else Path(__file__).resolve().parent
DATA_DIR = HERE.parent / "data" / "260523_house"
IMG_DIR = DATA_DIR / "images"
OUT_DIR = DATA_DIR / "keyframes"

DETECTORS = ["sift", "orb", "akaze", "brisk"]

# Video meta — used to derive timestamps (one frame_*.jpg per 3rd source frame).
SOURCE_FPS = 359 / 12     # from ffprobe on sample.mp4
SAMPLE_STRIDE = 3         # 01_make_frame_images.py extracted every 3rd frame

# %% [markdown]
# ## 셋업

# %%
images = sorted(IMG_DIR.glob("frame_*.jpg"))
assert images, f"no frames found under {IMG_DIR}"
print(f"found {len(images)} frames")

# Probe the first image for size to set up assumed intrinsics.
probe = cv2.imread(str(images[0]), cv2.IMREAD_COLOR)
assert probe is not None, f"failed to read {images[0]}"
H, W = probe.shape[:2]
camera = Camera.assumed_pinhole(W, H)
print(f"image size: {W}x{H}")
print(f"assumed K =\n{camera.K}")

store = KeyframeStore(root=OUT_DIR, dataset="260523_house", camera=camera)
detectors = {name: DETECTOR_FACTORIES[name]() for name in DETECTORS}
for det in detectors.values():
    store.register_detector(det.meta)

# %% [markdown]
# ## 검출 루프

# %%
counts: dict[str, int] = {n: 0 for n in DETECTORS}
t0 = time.perf_counter()

for i, path in enumerate(images):
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        print(f"[skip] failed to read {path.name}")
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    src_frame_idx = i * SAMPLE_STRIDE
    timestamp = src_frame_idx / SOURCE_FPS
    kf = Keyframe(
        id=path.stem,
        image_path=str(path.relative_to(DATA_DIR)),
        timestamp=timestamp,
    )

    for name, det in detectors.items():
        fs = det.detect_and_compute(gray)
        kf.features[name] = fs
        counts[name] += len(fs)

    store.add_frame(kf)

    if (i + 1) % 25 == 0 or i == len(images) - 1:
        per_det = "  ".join(f"{n}={len(kf.features[n])}" for n in DETECTORS)
        print(f"  [{i + 1:>3}/{len(images)}] {path.name}  {per_det}")

dt = time.perf_counter() - t0
print(f"\ndone in {dt:.1f}s")
for n in DETECTORS:
    print(f"  {n:5s}: total {counts[n]} kp  (avg {counts[n] / len(images):.1f}/frame)")

# %% [markdown]
# ## 저장 (manifest + 사이드카)

# %%
manifest_path = store.save()
print(f"manifest -> {manifest_path}")
print(f"sidecars under {OUT_DIR / 'features'}")

# %% [markdown]
# ## Round-trip sanity check

# %%
reloaded = KeyframeStore.load(OUT_DIR)
print(f"reloaded {len(reloaded.frames)} frames, detectors: {list(reloaded.detectors)}")
first = reloaded.frames[0]
for name in DETECTORS:
    fs = first[name]
    print(
        f"  {name:5s}  count={len(fs):>4d}  "
        f"kp.shape={fs.keypoints.shape}  "
        f"desc.shape={fs.descriptors.shape}  desc.dtype={fs.descriptors.dtype}"
    )
