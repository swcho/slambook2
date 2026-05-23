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
| `keyframe.py` | Library: `Camera`, `FeatureSet`, `Keyframe`, `KeyframeStore`, detector adapters | — |

Pipeline order: `01 → 02 → 03 → 04 → 05`. Scripts ≥ 03 only read the keyframe
store, so they can be re-run independently.

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
```

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
└── matches/                             # written by 04_match_pairwise.py
    ├── pairwise_counts.png              #   4-detector overlay (lin scale)
    ├── match_counts_by_detector.png     #   5_plot — lin + log, zero-runs shaded
    ├── sample_match_<detector>.jpg      #   drawMatches example pair
    └── <detector>/
        ├── pairwise_matches.npz         #   CSR-style match index (see below)
        └── pairwise_counts.csv          #   per-pair counts
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
