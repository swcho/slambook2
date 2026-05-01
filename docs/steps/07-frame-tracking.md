# Step 07 — Frame Tracking (LK prev → curr)

## 학습 목표

- [x] 시간축 LK가 공간축 LK(Step 4)와 같은 알고리즘이지만 입력 페어/초기치가 다름을 이해
- [x] 초기치 전략(none vs velocity prior)이 LK 수렴 라운드에 미치는 영향
- [x] map projection을 prior로 쓸 때의 가치(Step 8 PnP 후 자연 합류)

## C++ 원본 매핑

- 파일: `ch13/src/frontend.cpp::Frontend::TrackLastFrame`
- 핵심: `cv::calcOpticalFlowPyrLK`를 prev frame keypoints + curr frame에 호출. 책은 MapPoint가 있는 keypoint에 대해 그 3D 점을 현재 카메라 frame으로 투영한 좌표를 초기치로 사용.

## UI

- ParamPanel: prev/curr frame index · GFTT params · init strategy(none / velocity) · velocity dx/dy slider · LK params · maxIter sweep
- Input View: prev frame + GFTT overlay
- Output View:
  - **prev → curr 매칭선 오버레이** + 통계
  - **maxIter sweep chart** — 1..30 iter 각각의 mean residual + tracked count
- VerifyGate(자동 5건): WASM · prev/curr/calib · tracked ≥ 50 · mean dx ≈ −7 px (GT ± 2) · mean |dy| ≤ 2 px

## 알고리즘

- [x] 신규 C++ 없음 — features WASM(Step 3/4)이 `useInitialFlow + initialPts` 노출하므로 재사용
- [x] init strategy: `none` (`useInitialFlow=false`)
- [x] init strategy: `velocity` (사용자가 dx/dy 슬라이더로 등속도 prior 부여)
- [ ] init strategy: `map-projection` — Step 8 PnP가 상대 pose를 만들어내면 자연 합류 (Step 13에서 통합)

## 가속 경로

- [x] CPU scalar (Step 4와 동일)
- [ ] CPU SIMD / MT / WebGL / WebGPU — Phase C+ deferred

## 검증

- [x] 자동 수치 게이트 — VerifyGate 5건
- [x] 시각 수동 게이트 — `velocity` 초기치를 GT(-7, 0)에서 벗어나게 했을 때 추적 실패점 ↑ 관찰
- [x] 원본 C++ 결과와의 diff — OpenCV 직접 호출이라 일치 (단, map projection prior는 Step 13에서 합류)

## 학습 노트

Step 4(공간축 LK, 좌→우)와 Step 7(시간축 LK, prev→curr)은 OpenCV 입장에서 동일한 호출이다. 차이는 두 가지 — (1) **입력 페어**: 같은 시각의 두 카메라 vs 같은 카메라의 두 시각, (2) **초기치 전략**: stereo는 좌측 좌표 자체(disparity ≈ 0 가정), temporal은 직전 위치(velocity ≈ 0 가정) 또는 map projection.

KITTI mini의 합성 fixture는 frame당 14 px 좌측 이동 → 0.5× downsample 시 GT dx = −7 px. **velocity 슬라이더의 default를 −7로 두면** 학습자가 "GT prior와 일치할 때 tracking이 1 iter에 수렴" → "GT에서 벗어날 때 max iter까지 가서 수렴 또는 실패" 곡선의 plateau를 직접 관찰할 수 있다.

`map-projection`을 disabled chip으로 두고 Step 8 완료 후 자연 합류를 의도한 이유 — Step 7만 단독으로 있을 때 임의의 상대 pose를 손으로 입력하는 "fake projection"은 학습 가치가 낮음. Step 13의 풀 파이프라인에서 ch13의 `relative_motion_ * last_pose` init이 그대로 동작.

### 의도된 실패

- velocity dx/dy를 GT에서 크게 벗어난 값(예: dx=+20)으로 → tracking 거의 모두 실패, mean residual 폭증.
- prev frame을 텍스처 부족 영역으로 → keypoints가 cluster화, LK가 collapse.
