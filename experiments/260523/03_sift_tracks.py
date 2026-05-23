# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # SIFT — 다중 프레임 feature track
#
# 모든 연속 프레임 쌍에 대해 Lowe ratio match를 구한 뒤,
# `(frame_idx, keypoint_idx)` 노드 위에서 union-find로 track을 잇는다.
#
# 길이 L 인 track은 같은 물리적 feature가 L 프레임 동안 매칭되었음을 의미.
#
# Outputs
# - `data/20fps_real_img_archieve/sift_tracks/track_length_hist.png`
# - `data/20fps_real_img_archieve/sift_tracks/tracks_summary.csv`

# %%
import csv
from collections import Counter
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

HERE = Path.cwd() if "__file__" not in globals() else Path(__file__).resolve().parent
IMG_DIR = HERE.parent / "data" / "20fps_real_img_archieve"
FEAT_DIR = IMG_DIR / "sift_feat"
OUT_DIR = IMG_DIR / "sift_tracks"

RATIO = 0.75


# %% [markdown]
# ## Union-Find 자료구조

# %%
class UnionFind:
    def __init__(self) -> None:
        self.parent: dict[tuple[int, int], tuple[int, int]] = {}

    def find(self, x: tuple[int, int]) -> tuple[int, int]:
        self.parent.setdefault(x, x)
        root = x
        while self.parent[root] != root:
            root = self.parent[root]
        while self.parent[x] != root:
            self.parent[x], x = root, self.parent[x]
        return root

    def union(self, a: tuple[int, int], b: tuple[int, int]) -> None:
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb


# %%
def load_desc(path: Path) -> np.ndarray:
    with np.load(path) as z:
        return z["descriptors"].astype(np.float32)


def good_pairs(d1: np.ndarray, d2: np.ndarray, matcher: cv2.BFMatcher) -> list[tuple[int, int]]:
    if len(d1) < 2 or len(d2) < 2:
        return []
    knn = matcher.knnMatch(d1, d2, k=2)
    return [(m.queryIdx, m.trainIdx) for m, n in knn if m.distance < RATIO * n.distance]


# %% [markdown]
# ## 셋업

# %%
OUT_DIR.mkdir(exist_ok=True)

feat_paths = sorted(FEAT_DIR.glob("scene_avg_*.npz"))
assert len(feat_paths) >= 2, f"need >=2 feature files in {FEAT_DIR}"
print(f"found {len(feat_paths)} feature files")

matcher = cv2.BFMatcher(cv2.NORM_L2)
uf = UnionFind()

# %% [markdown]
# ## Pairwise match → union-find

# %%
prev_desc = load_desc(feat_paths[0])
total_links = 0
for i in range(len(feat_paths) - 1):
    curr_desc = load_desc(feat_paths[i + 1])
    pairs = good_pairs(prev_desc, curr_desc, matcher)
    for q, t in pairs:
        uf.union((i, q), (i + 1, t))
    total_links += len(pairs)
    if (i + 1) % 10 == 0 or i == len(feat_paths) - 2:
        print(f"  [{i + 1:>3}/{len(feat_paths) - 1}] linked {len(pairs)} pairs")
    prev_desc = curr_desc

# %% [markdown]
# ## Track 집계 (root별 그룹핑)

# %%
tracks: dict[tuple[int, int], list[tuple[int, int]]] = {}
for node in uf.parent:
    tracks.setdefault(uf.find(node), []).append(node)

# track length = 트랙이 걸쳐있는 distinct frame 수
lengths = [len({f for f, _ in nodes}) for nodes in tracks.values()]
length_hist = Counter(lengths)
lengths_arr = np.array(lengths)

print(f"total tracks: {len(tracks)}  (from {total_links} pairwise links)")
print(f"  L>=2 : {int((lengths_arr >= 2).sum())}")
print(f"  L>=5 : {int((lengths_arr >= 5).sum())}")
print(f"  L>=10: {int((lengths_arr >= 10).sum())}")
print(f"  max L: {int(lengths_arr.max())}")

# %% [markdown]
# ## CSV 저장

# %%
csv_path = OUT_DIR / "tracks_summary.csv"
with csv_path.open("w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["track_length", "count"])
    for L in sorted(length_hist):
        w.writerow([L, length_hist[L]])
print(f"csv -> {csv_path}")

# %% [markdown]
# ## Track length histogram

# %%
max_L = int(lengths_arr.max())
fig, ax = plt.subplots(figsize=(10, 4))
bins = np.arange(1.5, max_L + 1.5)  # integer-centered bins starting at 2
ax.hist(lengths_arr[lengths_arr >= 2], bins=bins, edgecolor="black")
ax.set_yscale("log")
ax.set_xlabel("track length (# frames feature persists)")
ax.set_ylabel("track count (log)")
ax.set_title(
    f"SIFT feature tracks  |  tracks(L>=2)={int((lengths_arr >= 2).sum())}  "
    f"max L={max_L}  median(L>=2)={int(np.median(lengths_arr[lengths_arr >= 2]))}"
)
ax.grid(alpha=0.3, axis="y")
fig.tight_layout()
png_path = OUT_DIR / "track_length_hist.png"
fig.savefig(png_path, dpi=120)
plt.show()

print(f"plot -> {png_path}")
