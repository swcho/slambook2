"""Loop-closure detection: BoW retrieval + essential-matrix geometry verification.

Pipeline
--------
1. Train a visual vocabulary by k-means over a sample of detector descriptors
   pulled from every keyframe in the store.
2. Encode each keyframe as a TF-IDF-weighted, L2-normalized BoW histogram.
3. For each keyframe `i`, retrieve the top-K most similar frames `j` (cosine
   similarity), excluding the temporal neighborhood `|i - j| < min_separation`.
4. Geometry-verify each `(i, j)` candidate with Lowe-ratio matching followed
   by `EssentialMatrixEstimator`. Surviving candidates become `LoopClosure`
   edges, ready to feed a pose-graph optimizer.

Notes
-----
- Float descriptors (e.g. SIFT) cluster cleanly with vanilla k-means. Binary
  descriptors (ORB/AKAZE/BRISK) work too with this implementation because we
  cast to float; for stronger binary retrieval one would normally use a
  DBoW2-style k-medoids over Hamming distance.
- The returned `RelativePose` keeps the usual `‖t‖ = 1` monocular convention.
  Loop-closure edges therefore feed Sim(3) pose-graph optimization most
  naturally — pure SE(3) PGO with unit-norm `t` is ill-posed.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from scipy.cluster.vq import kmeans2, vq

from keyframe import KeyframeStore, matcher_for
from pose import EssentialMatrixEstimator, RelativePose

SCHEMA_VERSION = "1.0"


# --------------------------------------------------------------------------- #
# Config & data
# --------------------------------------------------------------------------- #


@dataclass
class LoopClosureConfig:
    detector: str = "sift"               # float descriptors retrieve best
    vocab_size: int = 512                # k-means clusters
    vocab_sample_per_frame: int = 200    # cap descriptors per frame fed to k-means
    top_k: int = 5                       # retrieval candidates per query
    min_separation: int = 30             # |i - j| >= this; suppress temporal neighbors
    ratio: float = 0.75                  # Lowe ratio for geometry verification
    min_inliers: int = 30                # essential-matrix inlier accept threshold
    seed: int = 0


@dataclass
class LoopClosure:
    """A geometry-verified loop-closure edge. frame_a < frame_b."""

    frame_a: int
    frame_b: int
    similarity: float                    # cosine sim of TF-IDF BoW vectors
    rp: RelativePose                     # T_{b<-a}, unit-norm t, with inlier_idx


# --------------------------------------------------------------------------- #
# Vocabulary + BoW
# --------------------------------------------------------------------------- #


def build_vocabulary(store: KeyframeStore, cfg: LoopClosureConfig) -> np.ndarray:
    """Train a visual vocabulary; returns (K, D) centroid matrix."""
    rng = np.random.default_rng(cfg.seed)
    samples = []
    for kf in store.frames:
        d = kf.features[cfg.detector].descriptors
        if len(d) == 0:
            continue
        n = min(len(d), cfg.vocab_sample_per_frame)
        idx = rng.choice(len(d), n, replace=False)
        samples.append(d[idx].astype(np.float32))
    X = np.vstack(samples)
    # k-means++ init + Lloyd iterations. 'minit="++"' is the modern default.
    centroids, _ = kmeans2(X, cfg.vocab_size, iter=20, minit="++", seed=cfg.seed)
    return centroids.astype(np.float32)


def bow_histograms(store: KeyframeStore, cfg: LoopClosureConfig,
                   vocab: np.ndarray) -> np.ndarray:
    """L2-normalized TF-IDF BoW vectors, one per keyframe. Returns (N, K)."""
    N, K = len(store.frames), len(vocab)
    raw = np.zeros((N, K), dtype=np.float32)
    for i, kf in enumerate(store.frames):
        d = kf.features[cfg.detector].descriptors.astype(np.float32)
        if len(d):
            words, _ = vq(d, vocab)
            np.add.at(raw[i], words, 1.0)
    df = (raw > 0).sum(axis=0)                          # document frequency per word
    idf = np.log((N + 1) / (df + 1)) + 1.0
    tf = raw / np.maximum(raw.sum(axis=1, keepdims=True), 1.0)
    tfidf = tf * idf
    norms = np.linalg.norm(tfidf, axis=1, keepdims=True)
    return tfidf / np.maximum(norms, 1e-12)


# --------------------------------------------------------------------------- #
# Retrieval
# --------------------------------------------------------------------------- #


def retrieve_candidates(histograms: np.ndarray,
                        cfg: LoopClosureConfig) -> list[tuple[int, int, float]]:
    """Return (i, j, sim) candidates with j > i + min_separation; top_k per query."""
    N = len(histograms)
    sims = histograms @ histograms.T                    # (N, N) — small N here
    cands: list[tuple[int, int, float]] = []
    for i in range(N):
        scores = sims[i].copy()
        lo = max(0, i - cfg.min_separation + 1)
        hi = min(N, i + cfg.min_separation)
        scores[lo:hi] = -np.inf
        for j in np.argsort(-scores)[: cfg.top_k]:
            if j > i and scores[j] > 0:
                cands.append((i, int(j), float(scores[j])))
    return cands


# --------------------------------------------------------------------------- #
# Geometry verification
# --------------------------------------------------------------------------- #


def verify(store: KeyframeStore, cands: list[tuple[int, int, float]],
           cfg: LoopClosureConfig) -> list[LoopClosure]:
    matcher = matcher_for(store.detectors[cfg.detector])
    estimator = EssentialMatrixEstimator(min_inliers_pose=cfg.min_inliers)
    K = store.camera.K
    closures: list[LoopClosure] = []

    for i, j, sim in cands:
        fs_a = store.frames[i].features[cfg.detector]
        fs_b = store.frames[j].features[cfg.detector]
        if len(fs_a) < 2 or len(fs_b) < 2:
            continue
        knn = matcher.knnMatch(fs_a.descriptors, fs_b.descriptors, k=2)
        qi, ti = [], []
        for pair in knn:
            if len(pair) < 2:
                continue
            m, n = pair
            if m.distance < cfg.ratio * n.distance:
                qi.append(m.queryIdx)
                ti.append(m.trainIdx)
        if len(qi) < cfg.min_inliers:
            continue

        pts_a = fs_a.xy[np.asarray(qi)].astype(np.float32)
        pts_b = fs_b.xy[np.asarray(ti)].astype(np.float32)
        rp = estimator.estimate(pts_a, pts_b, K, frame_a=i, frame_b=j)
        if rp.success:
            closures.append(LoopClosure(i, j, sim, rp))
    return closures


# --------------------------------------------------------------------------- #
# End-to-end + JSON I/O
# --------------------------------------------------------------------------- #


def detect_loop_closures(store: KeyframeStore,
                         cfg: LoopClosureConfig | None = None
                         ) -> tuple[list[LoopClosure], np.ndarray]:
    cfg = cfg or LoopClosureConfig()
    vocab = build_vocabulary(store, cfg)
    hist = bow_histograms(store, cfg, vocab)
    cands = retrieve_candidates(hist, cfg)
    closures = verify(store, cands, cfg)
    return closures, hist


def save_loop_closures(closures: list[LoopClosure], cfg: LoopClosureConfig,
                       path: Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    out = {
        "version": SCHEMA_VERSION,
        "detector": cfg.detector,
        "config": {
            "vocab_size": cfg.vocab_size, "top_k": cfg.top_k,
            "min_separation": cfg.min_separation, "ratio": cfg.ratio,
            "min_inliers": cfg.min_inliers,
        },
        "edges": [
            {
                "frame_a": c.frame_a, "frame_b": c.frame_b,
                "similarity": c.similarity,
                "n_matches": c.rp.n_matches,
                "n_inliers_model": c.rp.n_inliers_model,
                "n_inliers_pose": c.rp.n_inliers_pose,
                "R": c.rp.R.tolist(), "t": c.rp.t.tolist(),
                "inlier_idx": c.rp.inlier_idx.tolist(),
            }
            for c in closures
        ],
    }
    with path.open("w") as f:
        json.dump(out, f, indent=2)
    return path


def load_loop_closures(path: Path) -> list[LoopClosure]:
    with Path(path).open() as f:
        d = json.load(f)
    if d.get("version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported loop-closure schema: {d.get('version')!r}")
    closures: list[LoopClosure] = []
    for e in d["edges"]:
        rp = RelativePose(
            frame_a=e["frame_a"], frame_b=e["frame_b"],
            success=True, method="loop_closure",
            n_matches=e["n_matches"],
            n_inliers_model=e["n_inliers_model"],
            n_inliers_pose=e["n_inliers_pose"],
            R=np.array(e["R"], dtype=np.float64),
            t=np.array(e["t"], dtype=np.float64),
            inlier_idx=np.array(e["inlier_idx"], dtype=np.int32),
        )
        closures.append(LoopClosure(e["frame_a"], e["frame_b"], e["similarity"], rp))
    return closures
