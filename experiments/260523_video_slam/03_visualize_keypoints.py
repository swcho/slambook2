# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Keyframe 시각화 — detector별 keypoint overlay
#
# `KeyframeStore` 에서 몇 개 sample frame을 골라 각 detector의 keypoint를
# `cv2.drawKeypoints(... DRAW_RICH_KEYPOINTS)` 로 overlay 하고 grid로 비교한다.
#
# 출력
# - `data/260523_house/keyframes/viz/sample_keypoints.png` — 4 프레임 × 4 detector
# - `data/260523_house/keyframes/viz/<frame_id>_<detector>.jpg` — 개별 overlay

# %%
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from keyframe import KeyframeStore

HERE = Path.cwd() if "__file__" not in globals() else Path(__file__).resolve().parent
DATA_DIR = HERE.parent / "data" / "260523_house"
STORE_DIR = DATA_DIR / "keyframes"
VIZ_DIR = STORE_DIR / "viz"

# Representative frames spanning dense → sparse → recovery.
SAMPLE_IDS = ["frame_00001", "frame_00075", "frame_00150", "frame_00250", "frame_00321"]
DETECTORS = ["sift", "orb", "akaze", "brisk"]

# %% [markdown]
# ## 셋업

# %%
VIZ_DIR.mkdir(exist_ok=True)
store = KeyframeStore.load(STORE_DIR)
print(f"loaded {len(store.frames)} frames, detectors: {list(store.detectors)}")

frames_by_id = {kf.id: kf for kf in store.frames}
missing = [fid for fid in SAMPLE_IDS if fid not in frames_by_id]
assert not missing, f"missing frames in store: {missing}"


# %% [markdown]
# ## Detector별 overlay 생성

# %%
def draw_overlay(img_bgr: np.ndarray, kf, detector: str) -> np.ndarray:
    kps = kf[detector].to_cv_keypoints()
    return cv2.drawKeypoints(
        img_bgr, kps, None,
        color=(0, 255, 0),
        flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
    )


# Save individual JPGs for each (frame, detector).
for fid in SAMPLE_IDS:
    kf = frames_by_id[fid]
    img = cv2.imread(str(DATA_DIR / kf.image_path), cv2.IMREAD_COLOR)
    for det in DETECTORS:
        viz = draw_overlay(img, kf, det)
        out_path = VIZ_DIR / f"{fid}_{det}.jpg"
        cv2.imwrite(str(out_path), viz, [cv2.IMWRITE_JPEG_QUALITY, 85])
print(f"wrote {len(SAMPLE_IDS) * len(DETECTORS)} per-(frame,detector) overlays")


# %% [markdown]
# ## Grid 비교 (frames × detectors)

# %%
rows, cols = len(SAMPLE_IDS), len(DETECTORS)
fig, axes = plt.subplots(rows, cols, figsize=(3.2 * cols, 5.6 * rows))
if rows == 1:
    axes = axes[None, :]

for r, fid in enumerate(SAMPLE_IDS):
    kf = frames_by_id[fid]
    img = cv2.imread(str(DATA_DIR / kf.image_path), cv2.IMREAD_COLOR)
    for c, det in enumerate(DETECTORS):
        viz = draw_overlay(img, kf, det)
        # downsample for display so the grid stays readable
        h, w = viz.shape[:2]
        viz_small = cv2.resize(viz, (w // 3, h // 3), interpolation=cv2.INTER_AREA)
        axes[r, c].imshow(cv2.cvtColor(viz_small, cv2.COLOR_BGR2RGB))
        axes[r, c].set_axis_off()
        n = len(kf[det])
        title = det.upper() if r == 0 else det
        axes[r, c].set_title(f"{title}  n={n}", fontsize=9)
    # row label = frame id on the left side
    axes[r, 0].text(
        -0.05, 0.5, fid, transform=axes[r, 0].transAxes,
        rotation=90, ha="right", va="center", fontsize=10,
    )

fig.suptitle("Keypoint overlays — DRAW_RICH_KEYPOINTS (size + orientation)", fontsize=12)
fig.tight_layout(rect=(0, 0, 1, 0.98))
grid_path = VIZ_DIR / "sample_keypoints.png"
fig.savefig(grid_path, dpi=140)
plt.show()
print(f"grid -> {grid_path}")
