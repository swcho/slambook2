# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Pairwise feature matching — SIFT / ORB / AKAZE / BRISK
#
# `KeyframeStore` 의 descriptor를 사용해 frame i ↔ i+1 사이에서
# Lowe ratio test 를 통과하는 매칭을 detector 별로 모두 계산한다.
#
# 각 detector마다 norm은 자동 선택 (`matcher_for(meta)` — L2 / Hamming).
#
# 출력
# - `data/260523_house/keyframes/matches/<det>/pairwise_matches.npz`
#     - `frame_a`, `frame_b`: (P,) 프레임 인덱스
#     - `offsets`         : (P+1,) 누적 매치 개수
#     - `pairs`           : (sum_M, 2) int32  [queryIdx, trainIdx]
#     - `distances`       : (sum_M,) float32
# - `data/260523_house/keyframes/matches/<det>/pairwise_counts.csv`
# - `data/260523_house/keyframes/matches/pairwise_counts.png`  — 4-detector 비교
# - `data/260523_house/keyframes/matches/sample_match_<det>.jpg`  — drawMatches 예시

# %%
import csv
import time
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from keyframe import KeyframeStore, matcher_for

HERE = Path.cwd() if "__file__" not in globals() else Path(__file__).resolve().parent
DATA_DIR = HERE.parent / "data" / "260523_house"
STORE_DIR = DATA_DIR / "keyframes"
OUT_DIR = STORE_DIR / "matches"

DETECTORS = ["sift", "orb", "akaze", "brisk"]
RATIO = 0.75                    # Lowe ratio test
SAMPLE_PAIR_IDX: int | None = None  # if None, picks the pair with the median match count


# %% [markdown]
# ## 셋업

# %%
OUT_DIR.mkdir(exist_ok=True)
store = KeyframeStore.load(STORE_DIR)
print(f"loaded {len(store.frames)} frames, detectors: {list(store.detectors)}")
n_pairs = len(store.frames) - 1
assert n_pairs >= 1


# %% [markdown]
# ## 헬퍼

# %%
def lowe_matches(d1: np.ndarray, d2: np.ndarray, matcher: cv2.BFMatcher
                 ) -> tuple[np.ndarray, np.ndarray]:
    """returns (pairs (M,2) int32, distances (M,) float32)."""
    if len(d1) < 2 or len(d2) < 2:
        return np.zeros((0, 2), np.int32), np.zeros((0,), np.float32)
    knn = matcher.knnMatch(d1, d2, k=2)
    good_q, good_t, good_d = [], [], []
    for pair in knn:
        if len(pair) < 2:
            continue
        m, n = pair
        if m.distance < RATIO * n.distance:
            good_q.append(m.queryIdx)
            good_t.append(m.trainIdx)
            good_d.append(m.distance)
    return (
        np.array(list(zip(good_q, good_t)), dtype=np.int32).reshape(-1, 2),
        np.array(good_d, dtype=np.float32),
    )


# %% [markdown]
# ## Detector별 matching 루프

# %%
counts: dict[str, np.ndarray] = {}
match_files: dict[str, Path] = {}
csv_paths: dict[str, Path] = {}

for det in DETECTORS:
    det_dir = OUT_DIR / det
    det_dir.mkdir(exist_ok=True)
    matcher = matcher_for(store.detectors[det])
    t0 = time.perf_counter()

    all_pairs, all_dists, offsets = [], [], [0]
    frame_a, frame_b = [], []
    rows: list[tuple[int, str, str, int, int, int]] = []

    for i in range(n_pairs):
        kf_a, kf_b = store.frames[i], store.frames[i + 1]
        da = kf_a[det].descriptors
        db = kf_b[det].descriptors
        pairs, dists = lowe_matches(da, db, matcher)

        all_pairs.append(pairs)
        all_dists.append(dists)
        offsets.append(offsets[-1] + len(pairs))
        frame_a.append(i)
        frame_b.append(i + 1)

        rows.append((i, kf_a.id, kf_b.id, len(da), len(db), len(pairs)))

    pairs_arr = np.concatenate(all_pairs, axis=0) if all_pairs else np.zeros((0, 2), np.int32)
    dists_arr = np.concatenate(all_dists, axis=0) if all_dists else np.zeros((0,), np.float32)

    npz_path = det_dir / "pairwise_matches.npz"
    np.savez_compressed(
        npz_path,
        frame_a=np.array(frame_a, np.int32),
        frame_b=np.array(frame_b, np.int32),
        offsets=np.array(offsets, np.int64),
        pairs=pairs_arr,
        distances=dists_arr,
        ratio=np.float32(RATIO),
    )
    match_files[det] = npz_path

    csv_path = det_dir / "pairwise_counts.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pair_idx", "frame_a", "frame_b", "n_kp_a", "n_kp_b", "n_good"])
        w.writerows(rows)
    csv_paths[det] = csv_path

    counts[det] = np.array([r[5] for r in rows], dtype=np.int32)
    dt = time.perf_counter() - t0
    c = counts[det]
    print(
        f"  {det:5s}  median={int(np.median(c)):>4d}  min={int(c.min()):>3d}  "
        f"max={int(c.max()):>4d}  total_matches={int(c.sum()):>6d}  "
        f"({dt:.1f}s)"
    )


# %% [markdown]
# ## 비교 plot — pair index vs. good-match count

# %%
fig, ax = plt.subplots(figsize=(12, 4.5))
for det in DETECTORS:
    ax.plot(counts[det], label=det.upper(), linewidth=1, alpha=0.85)
ax.set_xlabel("pair index (frame i -> i+1)")
ax.set_ylabel(f"good matches (Lowe ratio {RATIO})")
ax.set_title("Pairwise feature matches per detector")
ax.grid(alpha=0.3)
ax.legend(loc="upper right", fontsize=9)
fig.tight_layout()
plot_path = OUT_DIR / "pairwise_counts.png"
fig.savefig(plot_path, dpi=130)
plt.show()
print(f"plot -> {plot_path}")


# %% [markdown]
# ## Sample drawMatches — detector마다 한 쌍 시각화

# %%
# Choose a representative pair: one where every detector found a reasonable number
# of matches (use the median of min-across-detectors per pair).
per_pair_min = np.minimum.reduce([counts[d] for d in DETECTORS])
if SAMPLE_PAIR_IDX is None:
    candidate_idx = int(np.argsort(per_pair_min)[len(per_pair_min) // 2])
else:
    candidate_idx = SAMPLE_PAIR_IDX

kf_a = store.frames[candidate_idx]
kf_b = store.frames[candidate_idx + 1]
img_a = cv2.imread(str(DATA_DIR / kf_a.image_path), cv2.IMREAD_COLOR)
img_b = cv2.imread(str(DATA_DIR / kf_b.image_path), cv2.IMREAD_COLOR)
print(f"sample pair: {kf_a.id} -> {kf_b.id}  (per-detector mins {per_pair_min[candidate_idx]})")

MAX_DRAW = 80  # cap how many matches we draw, so overlays stay legible

for det in DETECTORS:
    with np.load(match_files[det]) as z:
        offsets = z["offsets"]
        pairs = z["pairs"]
        dists = z["distances"]
    lo, hi = int(offsets[candidate_idx]), int(offsets[candidate_idx + 1])
    pp = pairs[lo:hi]
    dd = dists[lo:hi]
    if len(pp) > MAX_DRAW:
        # keep the best (lowest-distance) matches for clarity
        order = np.argsort(dd)[:MAX_DRAW]
        pp, dd = pp[order], dd[order]

    kp_a = kf_a[det].to_cv_keypoints()
    kp_b = kf_b[det].to_cv_keypoints()
    cv_matches = [cv2.DMatch(int(q), int(t), float(d)) for (q, t), d in zip(pp, dd)]

    viz = cv2.drawMatches(
        img_a, kp_a, img_b, kp_b, cv_matches, None,
        matchColor=(0, 255, 0),
        singlePointColor=(0, 0, 255),
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
    )
    # downscale for storage (paired image is 2160×1920)
    h, w = viz.shape[:2]
    viz_small = cv2.resize(viz, (w // 2, h // 2), interpolation=cv2.INTER_AREA)
    out = OUT_DIR / f"sample_match_{det}.jpg"
    cv2.imwrite(str(out), viz_small, [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(f"  {det:5s}: drew {len(cv_matches)} / {hi - lo} matches -> {out.name}")


# %% [markdown]
# ## 요약

# %%
print("\nper-detector summary:")
for det in DETECTORS:
    c = counts[det]
    print(
        f"  {det:5s}  median={int(np.median(c)):>4d}  "
        f"≥10:{int((c >= 10).sum()):>3d}/{n_pairs}  "
        f"≥30:{int((c >= 30).sum()):>3d}/{n_pairs}  "
        f"==0:{int((c == 0).sum()):>3d}"
    )
print(f"\nartifacts under {OUT_DIR}")
