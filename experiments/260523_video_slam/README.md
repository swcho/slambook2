# 260523 video SLAM — keyframe pipeline

Phone-video → keyframe-store pipeline that produces a SLAM/SfM-ready feature
database for the `260523_house` capture (`experiments/data/260523_house/sample.mp4`).

## Scripts

| Script | Role | Output |
|---|---|---|
| `01_make_frame_images.py` | *(stub — frames are currently extracted via `ffmpeg` directly, see below)* | `data/260523_house/images/frame_*.jpg` |
| `02_extract_keyframes.py` | Detect features with SIFT / ORB / AKAZE / BRISK and write a keyframe store | `data/260523_house/keyframes/` |
| `keyframe.py` | Library: `Camera`, `FeatureSet`, `Keyframe`, `KeyframeStore`, detector adapters | — |

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

## On-disk format

Layout follows the OpenMVG / Kapture pattern — JSON manifest + binary sidecars
per frame. Descriptors are too bulky for inline JSON (SIFT alone ≈ 80 MB).

```
data/260523_house/keyframes/
├── keyframes.json                       # manifest (camera, detectors, frames)
└── features/
    ├── sift/frame_00001.npz             # keypoints + descriptors
    ├── orb/frame_00001.npz
    ├── akaze/frame_00001.npz
    └── brisk/frame_00001.npz
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
