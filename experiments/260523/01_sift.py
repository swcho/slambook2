# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # SIFT — 전체 프레임에 keypoint 검출
#
# `data/20fps_real_img_archieve/scene_avg_*.png` 모든 이미지에 SIFT를 적용해
# keypoint overlay와 keypoint/descriptor를 저장한다.
#
# Outputs
# - `data/20fps_real_img_archieve/sift_viz/<stem>_kp.png` : keypoint overlay
# - `data/20fps_real_img_archieve/sift_feat/<stem>.npz`   : keypoints + descriptors

# %%
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path.cwd() if "__file__" not in globals() else Path(__file__).resolve().parent
IMG_DIR = HERE.parent / "data" / "20fps_real_img_archieve"
VIZ_DIR = IMG_DIR / "sift_viz"
FEAT_DIR = IMG_DIR / "sift_feat"


# %%
def kp_to_array(kps: list[cv2.KeyPoint]) -> np.ndarray:
    """(N, 6) array of x, y, size, angle, response, octave."""
    return np.array(
        [(k.pt[0], k.pt[1], k.size, k.angle, k.response, k.octave) for k in kps],
        dtype=np.float32,
    )


# %% [markdown]
# ## 출력 디렉토리 준비 & 이미지 목록

# %%
VIZ_DIR.mkdir(exist_ok=True)
FEAT_DIR.mkdir(exist_ok=True)

images = sorted(IMG_DIR.glob("scene_avg_*.png"))
assert images, f"no images found under {IMG_DIR}"
print(f"found {len(images)} images")

# %% [markdown]
# ## SIFT 검출 루프

# %%
sift = cv2.SIFT_create()
total_kp = 0
t0 = time.perf_counter()

for i, path in enumerate(images):
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        print(f"[skip] failed to read {path.name}")
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    kps, desc = sift.detectAndCompute(gray, None)
    total_kp += len(kps)

    viz = cv2.drawKeypoints(
        img, kps, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
    )
    cv2.imwrite(str(VIZ_DIR / f"{path.stem}_kp.png"), viz)

    np.savez_compressed(
        FEAT_DIR / f"{path.stem}.npz",
        keypoints=kp_to_array(kps),
        descriptors=desc if desc is not None else np.zeros((0, 128), np.float32),
    )

    if (i + 1) % 10 == 0 or i == len(images) - 1:
        print(f"  [{i + 1:>3}/{len(images)}] {path.name}: {len(kps)} kp")

dt = time.perf_counter() - t0
print(
    f"\ndone: {len(images)} images, {total_kp} keypoints total "
    f"(avg {total_kp / max(len(images), 1):.1f}/img) in {dt:.1f}s"
)
print(f"viz   -> {VIZ_DIR}")
print(f"feat  -> {FEAT_DIR}")
