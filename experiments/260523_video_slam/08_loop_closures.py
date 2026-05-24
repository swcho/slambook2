# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Loop-closure detection
#
# BoW (TF-IDF over a learned visual vocabulary) → top-K retrieval per keyframe
# (with temporal exclusion) → essential-matrix geometry verification.
#
# 결과는 pose-graph optimization 의 long-range edge 로 사용. 결과가 0개라면
# 이 clip 에는 PGO 가 잡아낼 loop 가 없는 것이므로 PGO 인프라 작업을 미루는
# 의사결정 근거가 됨.
#
# 출력
# - `data/260523_house/keyframes/loop_closures/<detector>.json`
# - `data/260523_house/keyframes/loop_closures/<detector>_matrix.png` — N×N similarity
# - `data/260523_house/keyframes/loop_closures/<detector>_match_<i>_<j>.jpg` — top closures

# %%
import time
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from keyframe import KeyframeStore
from loop_closure import (
    LoopClosureConfig,
    build_vocabulary, bow_histograms, retrieve_candidates, verify,
    save_loop_closures,
)

HERE = Path.cwd() if "__file__" not in globals() else Path(__file__).resolve().parent
DATA_DIR = HERE.parent / "data" / "260523_house"
STORE_DIR = DATA_DIR / "keyframes"
OUT_DIR = STORE_DIR / "loop_closures"

DETECTOR = "sift"
TOP_DRAW = 8           # number of strongest closures to render as side-by-side overlays


# %% [markdown]
# ## 셋업

# %%
OUT_DIR.mkdir(parents=True, exist_ok=True)
store = KeyframeStore.load(STORE_DIR, load_features=True)
print(f"loaded {len(store.frames)} frames; detector={DETECTOR}")
cfg = LoopClosureConfig(detector=DETECTOR, top_k=5, min_separation=30, min_inliers=30)


# %% [markdown]
# ## Vocabulary + BoW + retrieval

# %%
t0 = time.perf_counter()
vocab = build_vocabulary(store, cfg)
print(f"  vocab: {vocab.shape}  ({time.perf_counter() - t0:.1f}s)")

t0 = time.perf_counter()
hist = bow_histograms(store, cfg, vocab)
print(f"  BoW:   {hist.shape}  ({time.perf_counter() - t0:.1f}s)")

t0 = time.perf_counter()
cands = retrieve_candidates(hist, cfg)
print(f"  retrieved {len(cands)} candidate pairs  ({time.perf_counter() - t0:.1f}s)")


# %% [markdown]
# ## Geometry verification

# %%
t0 = time.perf_counter()
closures = verify(store, cands, cfg)
print(f"  verified {len(closures)} / {len(cands)} candidates "
      f"({time.perf_counter() - t0:.1f}s)")

closures.sort(key=lambda c: -c.rp.n_inliers_pose)
for c in closures[:20]:
    print(f"  {c.frame_a:>3d} <-> {c.frame_b:>3d}  sim={c.similarity:.3f}  "
          f"in={c.rp.n_inliers_pose:>4d}  (gap {c.frame_b - c.frame_a})")

save_loop_closures(closures, cfg, OUT_DIR / f"{DETECTOR}.json")
print(f"saved -> {OUT_DIR / f'{DETECTOR}.json'}")


# %% [markdown]
# ## Similarity-matrix figure
#
# 대각선 = self-similarity (=1); off-diagonal hot spots are revisits.
# 빨간 점 = 검증 통과한 loop closure.

# %%
sims = hist @ hist.T
fig, ax = plt.subplots(figsize=(8, 7.5))
im = ax.imshow(sims, cmap="viridis", vmin=0, vmax=1, origin="upper")
plt.colorbar(im, ax=ax, label="cosine similarity")
ax.scatter([c.frame_b for c in closures], [c.frame_a for c in closures],
           s=18, edgecolor="red", facecolor="none", linewidths=1.2,
           label=f"loop closures (n={len(closures)})")
ax.set_xlabel("frame j")
ax.set_ylabel("frame i")
ax.set_title(f"{DETECTOR.upper()} TF-IDF BoW similarity (vocab={cfg.vocab_size})")
ax.legend(loc="upper right", fontsize=9)
fig.tight_layout()
mat_path = OUT_DIR / f"{DETECTOR}_matrix.png"
fig.savefig(mat_path, dpi=130)
plt.close(fig)
print(f"  similarity matrix -> {mat_path.name}")


# %% [markdown]
# ## 검증 통과한 상위 closure 들의 drawMatches overlay

# %%
for rank, c in enumerate(closures[:TOP_DRAW]):
    kf_a = store.frames[c.frame_a]
    kf_b = store.frames[c.frame_b]
    img_a = cv2.imread(str(DATA_DIR / kf_a.image_path), cv2.IMREAD_COLOR)
    img_b = cv2.imread(str(DATA_DIR / kf_b.image_path), cv2.IMREAD_COLOR)
    if img_a is None or img_b is None:
        print(f"  skip rank={rank}: image read failed")
        continue

    # rebuild the inlier point pairs from the stored inlier_idx + Lowe matches
    # (re-match because the verify step did the matching internally and
    # we didn't persist the queryIdx/trainIdx arrays)
    from keyframe import matcher_for
    matcher = matcher_for(store.detectors[DETECTOR])
    fs_a = kf_a.features[DETECTOR]
    fs_b = kf_b.features[DETECTOR]
    knn = matcher.knnMatch(fs_a.descriptors, fs_b.descriptors, k=2)
    qt: list[tuple[int, int, float]] = []
    for pair in knn:
        if len(pair) < 2:
            continue
        m, n = pair
        if m.distance < cfg.ratio * n.distance:
            qt.append((m.queryIdx, m.trainIdx, m.distance))
    qt_arr = np.array([(q, t) for q, t, _ in qt], dtype=np.int32)
    dist_arr = np.array([d for _, _, d in qt], dtype=np.float32)

    inlier_qt = qt_arr[c.rp.inlier_idx]
    inlier_d  = dist_arr[c.rp.inlier_idx]

    kp_a = fs_a.to_cv_keypoints()
    kp_b = fs_b.to_cv_keypoints()
    cv_matches = [cv2.DMatch(int(q), int(t), float(d))
                  for (q, t), d in zip(inlier_qt, inlier_d)]
    viz = cv2.drawMatches(
        img_a, kp_a, img_b, kp_b, cv_matches, None,
        matchColor=(0, 255, 0),
        singlePointColor=(0, 0, 255),
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
    )
    h, w = viz.shape[:2]
    viz_small = cv2.resize(viz, (w // 2, h // 2), interpolation=cv2.INTER_AREA)
    out = OUT_DIR / f"{DETECTOR}_match_{c.frame_a:03d}_{c.frame_b:03d}.jpg"
    cv2.imwrite(str(out), viz_small, [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(f"  rank={rank:>2d}  {c.frame_a:>3d}<->{c.frame_b:>3d}  "
          f"in={c.rp.n_inliers_pose:>4d}  -> {out.name}")
