# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 01 — KITTI mini dataset 로딩
#
# `src/lib/kitti.ts` 의 TS 로더와 동일한 결과를 Python 으로 재현한다.
# WASM 빌드와 같은 `public/datasets/kitti05-mini/` 데이터를 그대로 읽어
# `KittiCamera`, `StereoFrame` 자료구조로 노출한다.

# %%
import sys
from pathlib import Path

# scripts/ 에서 직접 실행할 때 myslam_ref 를 import 할 수 있도록 부모 디렉토리를 sys.path 에 추가.
_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from myslam_ref.dataset import load_kitti_mini
from myslam_ref.viz import draw_frame_pair  # noqa: F401  (used in notebook mode)

# %% [markdown]
# ## 1. calib.txt 파싱
#
# 기본 경로(`<repo>/ch13-wasm/public/datasets/kitti05-mini`)에서 calib + 5 frame 을 모두 로드.

# %%
ds = load_kitti_mini()
print(f"cameras: {len(ds.cameras)}")
for cam in ds.cameras:
    print(f"  P{cam.id}: fx={cam.fx:.4f} cx={cam.cx:.4f} t=({cam.t[0]:+.4f}, {cam.t[1]:+.4f}, {cam.t[2]:+.4f}) baseline={cam.baseline_m:.6f} m")

# %% [markdown]
# ## 2. K, P 행렬 확인 (camera0)

# %%
print("K (3x3) =")
print(ds.camera0.K)
print("P (3x4) =")
print(ds.camera0.P)

# %% [markdown]
# ## 3. 검증 assertion
#
# KITTI 05 시퀀스의 calib 와 mini fixture (5 frame, 1226×370) 에 대해 알려진 값을
# 미러: fx=707.0912, baseline≈0.5372 m, P0.t=(0,0,0), P1.t=(−0.537..,0,0).

# %%
cam0 = ds.camera0
cam1 = ds.camera1

assert len(ds.cameras) == 4, f"expected 4 P_k rows in calib.txt, got {len(ds.cameras)}"
assert cam0.fx == 707.0912
assert cam0.fy == 707.0912
assert cam0.cx == 601.8873
assert cam0.cy == 183.1104
assert abs(cam0.baseline_m) < 1e-9, f"P0 baseline should be 0, got {cam0.baseline_m}"
assert abs(cam1.baseline_m - 0.5371657) < 1e-4, f"P1 baseline 0.5372 m expected, got {cam1.baseline_m}"
# P1 stereo is a pure horizontal translation in the rig frame: t ≈ (-0.537, 0, 0).
assert abs(cam1.t[0] + 0.5371657) < 1e-4
assert abs(cam1.t[1]) < 1e-9
assert abs(cam1.t[2]) < 1e-9

# %% [markdown]
# ## 4. 스테레오 프레임 5 개 검사

# %%
assert len(ds.frames) == 5, f"expected 5 frames in kitti05-mini, got {len(ds.frames)}"
for frame in ds.frames:
    assert frame.left.dtype.name == "uint8"
    assert frame.right.dtype.name == "uint8"
    assert frame.left.shape == (370, 1226), f"frame {frame.index} left shape {frame.left.shape}"
    assert frame.right.shape == (370, 1226)
    assert frame.left.shape == frame.right.shape

# %% [markdown]
# ## 5. 시각화 (노트북 전용 — 스크립트 실행 시 inline display 가 없으면 생략)

# %%
if "ipykernel" in sys.modules or hasattr(sys, "ps1"):
    draw_frame_pair(ds.frames[0])

# %%
print("OK — step01 dataset reference matches kitti05-mini")
