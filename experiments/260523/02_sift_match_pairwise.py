# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # SIFT — 인접 프레임 pairwise matching
#
# `260523_sift.py`가 캐시한 descriptor를 읽어, frame i ↔ i+1 사이에서
# Lowe ratio test를 통과하는 good match 수를 집계한다.
#
# Outputs
# - `data/20fps_real_img_archieve/sift_match/pairwise_counts.csv`
# - `data/20fps_real_img_archieve/sift_match/pairwise_counts.png`

# %%
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

HERE = Path.cwd() if "__file__" not in globals() else Path(__file__).resolve().parent
IMG_DIR = HERE.parent / "data" / "20fps_real_img_archieve"
FEAT_DIR = IMG_DIR / "sift_feat"
OUT_DIR = IMG_DIR / "sift_match"

RATIO = 0.75  # Lowe's ratio test threshold


# %%
def load_desc(path: Path) -> np.ndarray:
    with np.load(path) as z:
        return z["descriptors"].astype(np.float32)


def count_good_matches(d1: np.ndarray, d2: np.ndarray, matcher: cv2.BFMatcher) -> int:
    if len(d1) < 2 or len(d2) < 2:
        return 0
    knn = matcher.knnMatch(d1, d2, k=2)
    return sum(1 for m, n in knn if m.distance < RATIO * n.distance)


# %% [markdown]
# ## 셋업

# %%
OUT_DIR.mkdir(exist_ok=True)

feat_paths = sorted(FEAT_DIR.glob("scene_avg_*.npz"))
assert len(feat_paths) >= 2, f"need >=2 feature files in {FEAT_DIR}"
print(f"found {len(feat_paths)} feature files")

matcher = cv2.BFMatcher(cv2.NORM_L2)

# %% [markdown]
# ## Pairwise matching 루프

# %%
rows: list[tuple[int, str, str, int, int, int]] = []

prev_desc = load_desc(feat_paths[0])
for i in range(len(feat_paths) - 1):
    curr_desc = load_desc(feat_paths[i + 1])
    n_good = count_good_matches(prev_desc, curr_desc, matcher)
    rows.append(
        (i, feat_paths[i].stem, feat_paths[i + 1].stem,
         len(prev_desc), len(curr_desc), n_good)
    )
    if (i + 1) % 10 == 0 or i == len(feat_paths) - 2:
        print(f"  [{i + 1:>3}/{len(feat_paths) - 1}] "
              f"{feat_paths[i].stem} -> {feat_paths[i+1].stem}: {n_good} matches")
    prev_desc = curr_desc

# %% [markdown]
# ## CSV 저장

# %%
csv_path = OUT_DIR / "pairwise_counts.csv"
with csv_path.open("w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["pair_idx", "frame_a", "frame_b", "n_kp_a", "n_kp_b", "n_good"])
    w.writerows(rows)
print(f"csv -> {csv_path}")

# %% [markdown]
# ## Plot — 프레임 인덱스 vs. good match 수

# %%
counts = np.array([r[5] for r in rows])
fig, ax = plt.subplots(figsize=(12, 4))
ax.plot(counts, marker=".", linewidth=1)
ax.set_xlabel("pair index (frame i -> i+1)")
ax.set_ylabel(f"good matches (Lowe ratio {RATIO})")
ax.set_title(f"Pairwise SIFT matches  |  median={int(np.median(counts))}  "
             f"min={counts.min()}  max={counts.max()}")
ax.grid(alpha=0.3)
fig.tight_layout()
png_path = OUT_DIR / "pairwise_counts.png"
fig.savefig(png_path, dpi=120)
plt.show()

print(f"\ndone: {len(rows)} pairs, "
      f"median={int(np.median(counts))} min={counts.min()} max={counts.max()}")
print(f"plot -> {png_path}")
