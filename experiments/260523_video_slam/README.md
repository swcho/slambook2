# 260523 video SLAM — keyframe pipeline

Phone-video → keyframe-store pipeline that produces a SLAM/SfM-ready feature
database for the `260523_house` capture (`experiments/data/260523_house/sample.mp4`).

## Scripts

| Script | Role | Output |
|---|---|---|
| `01_make_frame_images.py` | *(stub — frames are currently extracted via `ffmpeg` directly, see below)* | `data/260523_house/images/frame_*.jpg` |
| `02_extract_keyframes.py` | Detect features with SIFT / ORB / AKAZE / BRISK and write a keyframe store | `data/260523_house/keyframes/{keyframes.json, features/}` |
| `03_visualize_keypoints.py` | Per-detector keypoint overlay on representative frames | `data/260523_house/keyframes/viz/` |
| `04_match_pairwise.py` | i ↔ i+1 Lowe-ratio matching for all detectors; saves indices + counts + sample drawMatches | `data/260523_house/keyframes/matches/` |
| `05_plot_match_counts.py` | Linear + log plots of matches/frame, shading all-detector-zero runs | `data/260523_house/keyframes/matches/match_counts_by_detector.png` |
| `06_pose_estimation.py` | 5-point essential-matrix (RANSAC) + recoverPose per detector; builds segmented monocular trajectory, bridges failed pairs into world-frame groups, renders raw + bridged plots | `data/260523_house/keyframes/trajectories/` |
| `07_trajectory_animation.py` | Frame-by-frame MP4: current image + keypoint overlay (left) + accumulated XZ trajectory with current-camera marker (right) | `data/260523_house/keyframes/trajectories/<det>_bridged_animated.mp4` |
| `08_loop_closures.py` | TF-IDF BoW retrieval (vocab trained per-detector) + essential-matrix geometry verification; emits long-range loop-closure edges for downstream pose-graph optimization | `data/260523_house/keyframes/loop_closures/` |
| `09_pose_graph_optimization.py` | Sim(3) PGO via g2o: chain + bridge + loop-closure edges into a single graph, gauge-fixed at frame 0, Levenberg–Marquardt with Huber kernel; writes optimized trajectory + comparison plots | `data/260523_house/keyframes/trajectories/<det>_pgo.json` |
| `keyframe.py` | Library: `Camera`, `FeatureSet`, `Keyframe`, `KeyframeStore`, detector adapters | — |
| `pose.py` | Library: `RelativePose`, `EssentialMatrixEstimator`, `HomographyEstimator`, `Trajectory`, `bridge_segments`, `BridgeAttempt` | — |
| `loop_closure.py` | Library: `LoopClosureConfig`, `LoopClosure`, `build_vocabulary` / `bow_histograms` / `retrieve_candidates` / `verify`, `detect_loop_closures` | — |
| `sim3_pgo.py` | Library: `dedupe_loop_closures`, `optimize_sim3` (g2o-backed Sim(3) PGO), `trajectory_with_optimized_poses`, `PGOResult` | — |

Pipeline order: `01 → 02 → 03 → 04 → 05 → 06 → 07 → 08 → 09`. Scripts ≥ 03 only read the
keyframe store and pre-computed sidecars, so they can be re-run independently.

### Frame extraction (manual, until `01_*` is filled in)

```bash
ffmpeg -i data/260523_house/sample.mp4 \
       -vf "select=not(mod(n\,3))" -vsync vfr -q:v 2 \
       data/260523_house/images/frame_%05d.jpg
```

Yields 321 frames at ~10 fps from a 32 s clip (source 359/12 fps, stride 3).

### Keyframe extraction

```bash
python 02_extract_keyframes.py
```

Runtime: ~100 s for 321 frames × 4 detectors on the reference machine.

### Visualization + matching

```bash
python 03_visualize_keypoints.py   # overlays on 5 sample frames × 4 detectors
python 04_match_pairwise.py        # ~11 s for 320 pairs × 4 detectors
python 05_plot_match_counts.py     # reads matches/*.npz, writes the comparison plot
python 06_pose_estimation.py       # ~10 s for 320 pairs × 4 detectors, incl. bridging
python 07_trajectory_animation.py  # ~30 s; reads <det>_bridged.json (set DETECTOR at the top)
python 08_loop_closures.py         # ~4 min (vocab build is the slow part); set DETECTOR at top
python 09_pose_graph_optimization.py  # <1 s; reads <det>_bridged.json + loop_closures/<det>.json
```

`08_*` depends on `scipy.cluster.vq` (already required transitively). No
sklearn dependency. The vocab-build step dominates runtime (~4 min); cache
the centroids matrix to `.npy` if you iterate on retrieval params.

`09_*` depends on `g2o` (python binding). The optimizer itself runs in
milliseconds for 300-node graphs; the heavy lifting was already done by
`06_*` and `08_*`.

## On-disk format

Layout follows the OpenMVG / Kapture pattern — JSON manifest + binary sidecars
per frame. Descriptors are too bulky for inline JSON (SIFT alone ≈ 80 MB).

```
data/260523_house/keyframes/
├── keyframes.json                       # manifest (camera, detectors, frames)
├── features/                            # written by 02_extract_keyframes.py
│   ├── sift/frame_00001.npz             #   keypoints + descriptors
│   ├── orb/frame_00001.npz
│   ├── akaze/frame_00001.npz
│   └── brisk/frame_00001.npz
├── viz/                                 # written by 03_visualize_keypoints.py
│   ├── sample_keypoints.png             #   frames × detectors grid
│   └── <frame_id>_<detector>.jpg        #   per-(frame,detector) overlay
├── matches/                             # written by 04_match_pairwise.py
│   ├── pairwise_counts.png              #   4-detector overlay (lin scale)
│   ├── match_counts_by_detector.png     #   5_plot — lin + log, zero-runs shaded
│   ├── sample_match_<detector>.jpg      #   drawMatches example pair
│   └── <detector>/
│       ├── pairwise_matches.npz         #   CSR-style match index (see below)
│       └── pairwise_counts.csv          #   per-pair counts
├── trajectories/                        # written by 06_pose_estimation.py
│   ├── <detector>.json                  #   raw: poses, segments, per-pair pose results
│   ├── <detector>_xz.png                #   raw per-detector XZ trajectory
│   ├── <detector>_bridged.json          #   after bridging failed pairs (06 step 2)
│   ├── <detector>_bridged_xz.png        #   raw vs bridged side-by-side, color = world_group
│   ├── <detector>_bridged_animated.mp4  #   written by 07_trajectory_animation.py
│   ├── <detector>_bridged_animation_final.png
│   ├── <detector>_pgo.json              #   Sim(3)-optimized trajectory (09)
│   ├── <detector>_pgo_xz.png            #   bridged vs PGO side-by-side
│   ├── <detector>_pgo_scales.png        #   per-node optimized scale
│   └── comparison_xz.png                #   4-detector XZ overlay + Y-drift profile
└── loop_closures/                       # written by 08_loop_closures.py
    ├── <detector>.json                  #   verified long-range edges (i, j, R, t, inlier_idx)
    ├── <detector>_matrix.png            #   N×N BoW similarity heatmap; closures circled
    └── <detector>_match_<i>_<j>.jpg     #   drawMatches overlay for the top-K closures
```

### Manifest — `keyframes.json`

```json
{
  "version": "1.0",
  "dataset": "260523_house",
  "camera": {
    "model": "PINHOLE",
    "width": 1080,
    "height": 1920,
    "params": [1080.0, 1080.0, 540.0, 960.0]
  },
  "detectors": {
    "sift":  {"descriptor_dim": 128, "descriptor_dtype": "float32", "norm": "L2"},
    "orb":   {"descriptor_dim": 32,  "descriptor_dtype": "uint8",   "norm": "Hamming"},
    "akaze": {"descriptor_dim": 61,  "descriptor_dtype": "uint8",   "norm": "Hamming"},
    "brisk": {"descriptor_dim": 64,  "descriptor_dtype": "uint8",   "norm": "Hamming"}
  },
  "frames": [
    {
      "id": "frame_00001",
      "image_path": "images/frame_00001.jpg",
      "timestamp": 0.0,
      "features": {
        "sift": {"count": 5786, "sidecar": "features/sift/frame_00001.npz"},
        "orb":  {"count": 2000, "sidecar": "features/orb/frame_00001.npz"}
      },
      "pose_wc": [[...], [...], [...], [...]]
    }
  ]
}
```

Field notes:

- `version` — schema version (`"1.0"` today, bump on breaking changes).
- `camera` — **COLMAP camera-model convention** (`SIMPLE_PINHOLE`, `PINHOLE`,
  `SIMPLE_RADIAL`, `RADIAL`, `OPENCV`). Param order matches COLMAP exactly.
  Current capture uses fallback `fx = fy = W`, principal point at center
  (no calibration available yet).
- `detectors` — descriptor format per detector. `norm` selects the matcher
  (`L2` → `cv2.NORM_L2`, `Hamming` → `cv2.NORM_HAMMING`).
- `frames[i].features[det].sidecar` — path **relative to the store root**.
- `frames[i].timestamp` — seconds, derived from source fps × source frame index.
- `frames[i].pose_wc` — optional 4×4 world-from-camera matrix (absent until poses are estimated).

### Sidecar — `features/<detector>/<frame_id>.npz`

```
keypoints   : (N, 7) float32   # x, y, size, angle, response, octave, class_id
descriptors : (N, D) dtype     # dtype + D defined per-detector in manifest
```

The 7-column keypoint layout is a superset of `cv2.KeyPoint`, so round-trip
to/from OpenCV is lossless via `FeatureSet.to_cv_keypoints()`.

### Match sidecar — `matches/<detector>/pairwise_matches.npz`

CSR-style layout — one file holds all P pairs for a detector:

```
frame_a   : (P,)    int32     # source frame index (into store.frames)
frame_b   : (P,)    int32     # target frame index
offsets   : (P+1,)  int64     # cumulative match count; matches for pair p
                              #   live at [offsets[p] : offsets[p+1]]
pairs     : (M, 2)  int32     # [queryIdx, trainIdx] into each frame's keypoints
distances : (M,)    float32   # descriptor distance (L2 or Hamming, per detector)
ratio     : scalar  float32   # Lowe ratio used at extraction time
```

Read a single pair:

```python
with np.load("matches/sift/pairwise_matches.npz") as z:
    offs, pairs, dists = z["offsets"], z["pairs"], z["distances"]
lo, hi = int(offs[p]), int(offs[p + 1])
qt, dd = pairs[lo:hi], dists[lo:hi]            # (m, 2), (m,)
```

### Trajectory — `trajectories/<detector>.json`

Per-detector monocular trajectory built by `06_pose_estimation.py`. JSON
because it's human-inspectable. ~300 KB per detector for poses alone; with
`inlier_idx` (see below) it grows to ~2–3 MB per detector.

```json
{
  "version": "1.0",
  "detector": "sift",
  "method": "essential_ransac",
  "camera": { ... COLMAP camera ... },
  "frame_ids":  ["frame_00001", ..., "frame_00321"],
  "valid":      [true, true, ..., false, true],
  "segments":   [[0, 73], [74, 130], ...],
  "poses_wc":   [[[...4x4...]], null, [[...]], ...],
  "pairs": [
    {
      "frame_a": 0, "frame_b": 1, "success": true, "method": "essential_ransac",
      "n_matches": 329, "n_inliers_model": 280, "n_inliers_pose": 215,
      "R": [[...]], "t": [tx, ty, tz],
      "inlier_idx": [3, 7, 11, 14, ...]  // indices into the match array; len == n_inliers_pose
    },
    ...
  ],
  "extra": {}                            // bridged variant adds "world_group", "n_bridges"
}
```

Field notes:

- `poses_wc[i]` — 4×4 world-from-camera matrix, or `null` if the frame has no
  valid pose. NaN-equivalent for JSON.
- `segments[k] = [start, end)` — half-open intervals of contiguous valid
  frames. Across segments the world frames are **unrelated** — a failed pair
  breaks the chain, so each segment starts at its own identity origin.
- `pairs[i].t` is **unit-norm** (monocular scale ambiguity). Trajectory shape
  and rotation are meaningful; absolute distance is not.
- `pairs[i].R, t` define the camera-b-from-camera-a transform `T_{b←a}`
  (i.e. `x_b = R · x_a + t`).
- `pairs[i].inlier_idx` — indices into the pair's match array
  (`matches/<det>/pairwise_matches.npz[offsets[i]:offsets[i+1]]`) identifying
  which correspondences passed RANSAC + cheirality and produced `(R, t)`. Use
  with `store.frames[a/b][det].xy` to rebuild per-feature reprojection
  residuals for non-linear optimization (BA / pose-graph). `null` when
  `success=false`; empty array on pre-`inlier_idx` files (backwards-compat).

Accumulation rule used to fill `poses_wc`:
`T_wc[i+1] = T_wc[i] · inv(T_{b←a})  =  T_wc[i] · [R^T | -R^T t]`.

Rebuilding inlier observations for BA:

```python
with np.load(MATCH_DIR / det / "pairwise_matches.npz") as z:
    offsets, pairs_idx = z["offsets"], z["pairs"]

for p in traj.pairs:
    if not p.success:
        continue
    qt = pairs_idx[int(offsets[p.frame_a]):int(offsets[p.frame_a + 1])]
    inlier_qt = qt[p.inlier_idx]                                    # (n_in, 2)
    xy_a = store.frames[p.frame_a][det].xy[inlier_qt[:, 0]]         # (n_in, 2)
    xy_b = store.frames[p.frame_b][det].xy[inlier_qt[:, 1]]         # (n_in, 2)
    # → feed to pyceres / g2o / gtsam as reprojection-error residual blocks
```

### Bridging — `trajectories/<detector>_bridged.json`

`bridge_segments(traj, store)` re-runs feature matching + essential-matrix
estimation (with homography fallback) across each segment boundary, trying
skip-frame candidates `(eA - 1 - di, sB + dj)` for small `(di, dj)`. The
common case `(0, 0)` skips the single bad keyframe between two segments and
recovers the link; rarer wider offsets handle short blur runs.

If the link succeeds for boundary `(k, k+1)`, segment `k+1`'s poses are
transformed into segment `k`'s world frame and `world_group[k+1] = world_group[k]`.
Two segments share a world frame iff their `world_group` values match.

Bridging recovers **rotation + unit-norm translation direction** but **not
metric scale** — each pair contributes one unit step, just like in
`from_pairs`. The invalid frame between bridged segments stays invalid, so
`positions[]` still has NaN there; `Line2D` auto-breaks at that single frame,
but the surrounding poses are continuous.

Bridge poses set `inlier_idx` against the re-matched `pts_a/pts_b` arrays
built inside `_bridge_match`, but those match arrays are **not persisted** —
only the original i ↔ i+1 matches live in `pairwise_matches.npz`. To feed
bridge edges into BA, `_bridge_match` would need to also return the
`(queryIdx, trainIdx)` arrays for storage alongside the bridge `RelativePose`.

Schema additions (only in the `_bridged.json` variant):

- `method` — `"essential_ransac+bridge"`.
- `extra.world_group` — `list[int]` of length `len(segments)`. Segments sharing
  a value live in the same world frame.
- `extra.n_bridges` — count of successful boundary links.

### Loop closures — `loop_closures/<detector>.json`

`detect_loop_closures(store, cfg)` does TF-IDF BoW retrieval (visual vocabulary
trained per run by `scipy.cluster.vq.kmeans2`) followed by essential-matrix
geometry verification. Output is a flat list of long-range edges:

```json
{
  "version": "1.0",
  "detector": "sift",
  "config": { "vocab_size": 512, "top_k": 5, "min_separation": 30,
              "ratio": 0.75, "min_inliers": 30 },
  "edges": [
    {
      "frame_a": 91, "frame_b": 121, "similarity": 0.786,
      "n_matches": 1247, "n_inliers_model": 271, "n_inliers_pose": 154,
      "R": [[...]], "t": [tx, ty, tz],
      "inlier_idx": [...]
    },
    ...
  ]
}
```

Field notes:

- `frame_a < frame_b` always; pair indices are *not* contiguous, unlike
  `Trajectory.pairs`.
- `R, t` follow the same convention as `Trajectory.pairs[i]` — `T_{b←a}` with
  unit-norm `t`. The unit norm is a **scale conflict** against the chain's
  accumulated drift, which is exactly what Sim(3) PGO is designed to resolve.
- `inlier_idx` indexes into the **re-matched** descriptor list (Lowe ratio
  on the full descriptor sets of frames `a` and `b`), not into the
  consecutive-pair match arrays in `pairwise_matches.npz`. To recover the
  `(queryIdx, trainIdx)` for BA, re-run the same Lowe match — it's
  deterministic given identical `ratio`.

The `min_separation` threshold (default 30 frames ≈ 3 s @ 10 fps) suppresses
near-temporal candidates that would otherwise just look like the
consecutive-pair chain. Tune up for slow scenes, down for fast ones.

### Sim(3) PGO — `trajectories/<detector>_pgo.json`

`optimize_sim3(traj, closures)` builds a g2o `SparseOptimizer` with
`VertexSim3Expmap` nodes (one per valid frame, storing `Scw`) and three
edge populations:

- **chain edges** from `traj.pairs` where `success=True` — measurement
  `Sba = (R, t, 1)` from the relative pose, weighted by `n_inliers_pose`.
- **bridge edges** synthesized from `extra["world_group"]` — for every
  pair of adjacent segments sharing a world group, the relative `T_ba`
  derived from the bridged poses, with moderate (`0.5 * I`) info weight.
- **loop-closure edges** loaded from `loop_closures/<detector>.json`,
  passed through `dedupe_loop_closures(closures, nms_window=5)` to keep
  only the strongest edge per `(frame_a±w, frame_b±w)` cluster.

Gauge is fixed by `set_fixed(True)` on the first valid frame. Solver:
Levenberg–Marquardt with `LinearSolverEigenSim3`, Huber kernel
(`delta ≈ √7` ≈ 2.6 — one sigma on the 7-DoF Sim(3) tangent residual).

Schema additions on `_pgo.json` (on top of the bridged variant):

- `method` — appended with `"+sim3pgo"`.
- `poses_wc[i]` — optimized; the per-node scale is **baked into the
  translation column** so positions are scale-corrected, but the 3×3
  rotation block stays orthonormal. Downstream code that needs the raw
  Sim(3) transform should multiply the rotation block by `pgo_scales[i]`.
- `extra.pgo_scales` — `list[float]` of length N, optimized scale per node
  (1.0 for invalid frames).
- `extra.pgo_initial_chi2`, `extra.pgo_final_chi2`,
  `extra.pgo_n_chain`, `extra.pgo_n_bridge`, `extra.pgo_n_loop`.

**Why Sim(3) is required (not SE(3))**: every chain/bridge/loop-closure
edge in this pipeline carries `‖t‖ = 1` (monocular convention). Pure SE(3)
PGO cannot reconcile a loop-closure unit-norm `t` with the chain's
accumulated unit-norm steps; the residual is degenerate in the scale
direction. Sim(3) gives each node its own scale variable, and the
optimizer distributes scale around the loop to make the constraints
consistent.

**Data-limitation note**: PGO can only refine the connected component of
the graph that touches a loop closure. Segments in world groups with no
loop-closure edge keep `scale = 1.0` (their bridged initial pose). The
SIFT result here shows a 5× scale span [0.27, 1.33] in the loop region
and `s = 1.0` everywhere else — a faithful reflection of which parts of
the trajectory the data actually constrains, not a bug.

## Library API

```python
from keyframe import (
    Camera, KeyframeStore, Keyframe,
    DETECTOR_FACTORIES, matcher_for,
)

store = KeyframeStore.load("data/260523_house/keyframes")
kf = store.frames[0]
sift = kf["sift"]              # FeatureSet
sift.xy                        # (N, 2) coordinates
sift.descriptors               # (N, 128) float32
sift.to_cv_keypoints()         # list[cv2.KeyPoint]
store.camera.K                 # 3x3 intrinsic matrix

matcher = matcher_for(store.detectors["orb"])   # BFMatcher(NORM_HAMMING)
```

Creating a store from scratch:

```python
store = KeyframeStore(root=out_dir, dataset="...", camera=Camera.assumed_pinhole(W, H))
sift = DETECTOR_FACTORIES["sift"]()             # also: orb, akaze, brisk
store.register_detector(sift.meta)

kf = Keyframe(id="frame_00001", image_path="images/frame_00001.jpg")
kf.features["sift"] = sift.detect_and_compute(gray)
store.add_frame(kf)
store.save()                                    # writes manifest + sidecars
```

Pose estimation + trajectory:

```python
from pose import EssentialMatrixEstimator, Trajectory, bridge_segments

estimator = EssentialMatrixEstimator(threshold_px=1.0, min_inliers_pose=15)
pair_results = [estimator.estimate(pts_a, pts_b, K, frame_a=i, frame_b=i+1)
                for i, (pts_a, pts_b) in enumerate(pair_iter)]

traj = Trajectory.from_pairs(
    detector="sift", method=estimator.method, camera=store.camera,
    frame_ids=[kf.id for kf in store.frames], pairs=pair_results,
)
traj.save("trajectories/sift.json")
traj.positions            # (N, 3) — NaN where invalid
traj.segment_positions()  # list[(M, 3)] — one per contiguous segment

bridged, attempts = bridge_segments(traj, store)   # store has descriptors
bridged.extra["world_group"]   # list[int] — segments sharing a world frame
bridged.extra["n_bridges"]     # int — number of boundaries successfully linked
bridged.save("trajectories/sift_bridged.json")
for a in attempts:             # inspect why bridges succeeded/failed
    print(a)                   # e.g. <Bridge ok seg3->4 f161-163 homography:0,0 m=1320 in=1063>
```

## Why this format

- **No single JSON-native keyframe standard exists.** The closest established
  conventions are COLMAP (text/binary), OpenMVG (`sfm_data.json` + sidecar
  features), Kapture (NAVER LABS — text manifest + binary sidecars), and HLoc
  (HDF5). All of them separate bulky descriptors from the manifest.
- **COLMAP intrinsics + OpenMVG-style sidecars** gives us interop with the
  most common tooling without inventing a new camera model.
- **Per-detector sidecars** (rather than one combined file) keep the store
  composable — you can compute ORB later without rewriting SIFT data.

## Detectors

| Name  | Descriptor | Norm    | Typical use |
|-------|------------|---------|-------------|
| SIFT  | 128 × f32  | L2      | SfM / loop closure baseline |
| ORB   | 32 × u8    | Hamming | ORB-SLAM, real-time SLAM |
| AKAZE | 61 × u8    | Hamming | Nonlinear scale-space, blur-robust |
| BRISK | 64 × u8    | Hamming | Fast binary, scale/rotation invariant |

All are OpenCV-backed (no patented / contrib-only detectors).

## Results so far (260523_house, 321 frames, 320 pairs)

**Per-detector match counts** (Lowe ratio 0.75, frame i ↔ i+1)

| detector | median | ≥30 / 320 | ≥100 / 320 | ≥300 / 320 | zero pairs |
|---|---|---|---|---|---|
| SIFT  | 302 | 267 | 224 | 160 | 26 |
| ORB   | 589 | 267 | 243 | 202 | 31 |
| AKAZE | 615 | 278 | 247 | 211 | 27 |
| BRISK | 269 | 256 | 212 | 157 | 29 |

**All-detector failure windows** (pair index → frame ids) — these are the
unrecoverable gaps where every detector returns 0 matches simultaneously,
caused by a near-blank wall / motion-blur segment in the source clip:

| pair range | frames | width |
|---|---|---|
| 223–230 | `frame_00224 … frame_00232` | 8 |
| 243–254 | `frame_00244 … frame_00256` | 12 |
| 258–263 | `frame_00259 … frame_00265` | 6 |

AKAZE leads at every threshold ≥ 10; ORB is second by virtue of its 2000-kp
cap producing a flatter distribution. Any tracking pipeline on this clip
needs a re-init / loop-closure strategy across the three red windows above.

**Pose estimation** (5-point essential + RANSAC, `threshold_px=1.0`,
`min_inliers_pose=15`)

| detector | successful pairs | valid frames | # segments | longest segment | world groups (post-bridge) | gaps closed |
|---|---|---|---|---|---|---|
| SIFT  | 240 / 320 | 272 / 321 | 50 | 72 | 21 | 29 / 49 |
| ORB   | 245 / 320 | 275 / 321 | 47 | 49 | 27 | 20 / 46 |
| AKAZE | 258 / 320 | 285 / 321 | 37 | 93 | 20 | 17 / 36 |
| BRISK | 222 / 320 | 265 / 321 | 56 | 57 | 27 | 29 / 55 |

AKAZE again wins — fewest segments (37) and longest continuous segment
(93 frames). The big curved arc visible in the XZ plots corresponds to that
93-frame run; outside of it the trajectory shatters into many short segments,
mostly because of the three all-detector-zero windows already noted plus a
handful of geometry-degenerate pairs (pure rotation, planar scenes) where
`recoverPose` returns < 15 cheirality inliers.

`bridge_segments` recovers the single-bad-frame boundaries (most common
failure cause): the `(0, 0)` skip-the-bad-frame offset closes 17–29 gaps per
detector, collapsing 37–56 segments into 20–27 world groups. Remaining gaps
fall in the all-detector-zero windows or true geometry-degenerate pairs —
they are candidates for the PnP-based bridging or wider `skip_offsets`.

**Loop closures** (SIFT, vocab=512, top_k=5, min_separation=30, min_inliers=30)

- 47 verified loop closures from 585 retrieval candidates.
- After `dedupe_loop_closures(nms_window=5)`: **6 representative edges**, including
  the strongest 91 ↔ 121 (sim 0.79, 154 inliers) plus secondary clusters
  like 33 ↔ 101 and 39 ↔ 86 (longer-range revisits).
- The all-detector-zero windows (frames 224–265) show up as a dim band on
  the similarity matrix and produce zero closures, as expected.

**Sim(3) PGO** (SIFT, bridged + 6 loop-closure edges)

| metric | value |
|---|---|
| vertices | 272 (valid frames only) |
| chain / bridge / loop edges | 222 / 29 / 6 |
| χ² | **2141 → 24.84 (98.8% reduction)** |
| per-node scale span | **[0.27, 1.33]** — 5× spread |
| runtime (30 LM iterations) | 0.14 s |

The scale plot reveals the structure of the data: scales are pinned at 1.0
before the first loop closure (frames 0–30), collapse to ~0.3 in the loop
region (frames 30–130, where the chain claimed unit-norm steps but the
revisit proved those steps were 3× shorter), jump to ~1.33 in the next
world group (frames 130–225), and return to 1.0 for the final disconnected
group (frames 225+). World groups untouched by any loop closure cannot be
refined and keep their bridged initial pose — exactly what we'd expect.
