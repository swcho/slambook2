# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 03 — Feature detection (+ Step 4 stereo LK 미리보기)
#
# `wasm-src/spike/verify_features.ts` 의 검증 시나리오를 Python 으로 미러한다.
# 4 개 detector (GFTT/Harris/FAST/ORB) 가 합성 frame 에서 모두 ≥ 30 keypoint 를
# 내고, 마스크 배제와 LK 트래킹이 책과 동일한 의미로 동작하는지 확인.

# %%
import time

import cv2
import numpy as np

from myslam_ref.dataset import load_kitti_dataset
from myslam_ref.features import Detector, detect, render_synth_frame, track_lk
from myslam_ref.viz import draw_detectors_grid, draw_flow_field, draw_mask_overlay  # noqa: F401

# %% [markdown]
# ## 1. OpenCV 버전 확인 (WASM 빌드는 4.13)

# %%

print(f"opencv-python version: {cv2.__version__}")

# %% [markdown]
# ## 2. 합성 frame 생성 (renderFrame TS 와 동일한 합성 영상)

# %%
W, H = 1226, 370
frame0 = render_synth_frame(0, 0)
assert frame0.shape == (H, W)
assert frame0.dtype == np.uint8

# %% [markdown]
# ## 3. 4 detector 모두 ≥ 30 keypoint
#
# `verify_features.ts` 의 ``if (n < 30) fail(...)`` 게이트를 미러.

# %%
detectors = [
    ("GFTT", Detector.GFTT),
    ("Harris", Detector.HARRIS),
    ("FAST", Detector.FAST),
    ("ORB", Detector.ORB),
]
counts = {}
opts = dict(
    maxFeatures=200, qualityLevel=0.01, minDistance=20, blockSize=3,
    fastThreshold=20, nonmaxSuppression=True, orbScaleFactor=1.2, orbNLevels=8,
)
for name, algo in detectors:
    t0 = time.perf_counter()
    pts = detect(frame0, algo, **opts)
    dt = (time.perf_counter() - t0) * 1000
    counts[name] = pts.shape[0]
    print(f"  {name:<6} → {pts.shape[0]:4d} kps in {dt:6.1f} ms")
    assert pts.shape[0] >= 30, f"{name}: expected ≥ 30, got {pts.shape[0]}"

# %% [markdown]
# ## 4. Mask 배제 — 100×100 hole 안에 keypoint 0개

# %%
cx_hole, cy_hole = 588, 170  # marker 중심과 일치 (verify_features.ts:124)
mask = np.full((H, W), 255, dtype=np.uint8)
mask[max(0, cy_hole - 50):cy_hole + 50, max(0, cx_hole - 50):cx_hole + 50] = 0
masked = detect(frame0, Detector.GFTT, maxFeatures=200,
                qualityLevel=0.01, minDistance=20, blockSize=3, mask=mask)

in_hole = int(
    np.sum(
        (masked[:, 0] >= cx_hole - 50) & (masked[:, 0] < cx_hole + 50)
        & (masked[:, 1] >= cy_hole - 50) & (masked[:, 1] < cy_hole + 50)
    )
)
print(
    f"GFTT + 100×100 mask hole → {masked.shape[0]} kps, {in_hole} inside hole")
assert in_hole == 0, f"mask exclusion broken: {in_hole} keypoints inside hole"

# %% [markdown]
# ## 5. LK 트래킹 (Step 4 stereo matching 미리보기)
#
# 합성 stereo pair (frame 0 cam 0 → cam 1) 의 disparity 는 22 px.
# trackLK 가 ≥ 70 % 트래킹 + mean dx ≈ -22 를 만족해야 한다.

# %%
left0 = frame0
right0 = render_synth_frame(0, 1)
seeds = detect(left0, Detector.GFTT, maxFeatures=200,
               qualityLevel=0.01, minDistance=20)

t_lk = time.perf_counter()
tracked = track_lk(left0, right0, seeds, winSize=11, maxLevel=3,
                   maxIter=30, eps=0.01, useInitialFlow=False)
dt_lk = (time.perf_counter() - t_lk) * 1000

ok_mask = tracked[:, 2] > 0.5
ok_count = int(ok_mask.sum())
n = seeds.shape[0]
mean_dx = float((tracked[ok_mask, 0] - seeds[ok_mask, 0]
                 ).mean()) if ok_count else float("nan")
mean_dy = float((tracked[ok_mask, 1] - seeds[ok_mask, 1]
                 ).mean()) if ok_count else float("nan")
track_rate = ok_count / n if n else 0.0
print(
    f"LK left→right (no init) tracked {ok_count}/{n} ({track_rate * 100:.1f}%) in {dt_lk:.1f} ms")
print(f"  mean shift dx={mean_dx:+.2f} dy={mean_dy:+.2f} (expect ≈ -22, 0)")
assert track_rate >= 0.7, f"trackRate {track_rate:.1%} < 70%"
assert abs(mean_dx + 22) < 3, f"mean dx {mean_dx:.2f} not near -22"
assert abs(mean_dy) < 1, f"mean dy {mean_dy:.2f} not near 0"

# %% [markdown]
# ## 6. useInitialFlow regression
#
# GT 로 미리 시드를 주면 트래킹이 동등하거나 더 좋아야 한다.

# %%
initial_pts = seeds.copy()
initial_pts[:, 0] -= 22
initial_pts[:, 2] = 1.0
tracked_init = track_lk(
    left0, right0, seeds,
    winSize=11, maxLevel=3, maxIter=30, eps=0.01, useInitialFlow=True, initialPts=initial_pts,
)
ok_init = int((tracked_init[:, 2] > 0.5).sum())
ok_mask_init = tracked_init[:, 2] > 0.5
mean_dx_init = float((tracked_init[ok_mask_init, 0] -
                     seeds[ok_mask_init, 0]).mean()) if ok_init else float("nan")
print(
    f"LK with useInitialFlow tracked {ok_init}/{n} ({ok_init / n * 100:.1f}%), mean dx={mean_dx_init:+.2f}")
assert ok_init >= ok_count, f"useInitialFlow regressed: {ok_init} < {ok_count}"
assert abs(mean_dx_init +
           22) < 1.5, f"useInitialFlow dx {mean_dx_init:.2f} not near -22"

# %% [markdown]
# ## 7. KITTI mini 실데이터 smoke
#
# 합성 입력만으론 부족하니, 실제 frame 0 left 에 GFTT 를 돌려 합리적 개수가 나오는지 본다.

# %%
ds = load_kitti_dataset()
real_left0 = ds.frames[0].left
pts_real = detect(real_left0, Detector.GFTT, maxFeatures=200,
                  qualityLevel=0.01, minDistance=20)
print(f"GFTT on kitti05-mini frame 0 left → {pts_real.shape[0]} kps")
assert pts_real.shape[
    0] >= 50, f"expected ≥ 50 GFTT kps on real frame, got {pts_real.shape[0]}"

# %% [markdown]
# ## 8. 시각화 — 4 detector overlay + mask hole + stereo LK flow

# %%
detector_results = {}
for name, algo in detectors:
    detector_results[name] = detect(frame0, algo, **opts)
fig_d = draw_detectors_grid(frame0, detector_results)

# %%
fig_m = draw_mask_overlay(
    frame0, mask, title="GFTT mask hole (100×100 around marker)")

# %%
fig_f = draw_flow_field(
    left0, seeds[:, :2], tracked[:, :2], status=tracked[:, 2],
    title="stereo LK: left → right flow (rectified pair, dx ≈ -22)",
    step=2,
)

# %%
print("OK — step03 feature detection reference passes 4-detector + mask + LK gate")
