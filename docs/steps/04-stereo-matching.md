# Step 04 — Stereo Matching (LK pyramid)

## 학습 목표

- [x] Lucas-Kanade 광학 흐름의 윈도우 + pyramid 구조 이해
- [x] descriptor 매칭 vs flow 매칭의 핵심 차이(공간적 연속성 가정)
- [x] LK가 stereo rectified 쌍에서 동작하는 이유(좌→우 disparity는 horizontal-only)

## C++ 원본 매핑

- 파일: `ch13/src/frontend.cpp::Frontend::FindFeaturesInRight`
- 핵심: `cv::calcOpticalFlowPyrLK` — 좌측 keypoints에서 우측 위치를 찾는 시간축 LK 변형(공간축)
- 본 Step은 Phase C 게이트 spike를 통해 검증된 OpenCV 4.13.0의 LK 호출을 그대로 노출.

## UI

- ParamPanel: window size(7..31) · max levels(1..5) · max iterations(10..50) · eps · useInitialFlow toggle
- Input View: 좌측 이미지 + GFTT keypoints
- Output View: 좌→우 매칭선 오버레이 + 통계(매칭율, mean dx, mean |dy|)
- VerifyGate(자동 4건): WASM 로드 · left/right 로드 · 매칭율 ≥ 60% · mean |dy| ≤ 2 px · mean dx < 0(우측이 좌측보다 좌로)

## 알고리즘

- [x] LK pyramid (원본) — `cv::calcOpticalFlowPyrLK` via `bind_features.trackLK`
- [ ] ORB descriptor + BF Hamming — Phase C+ deferred
- [ ] SGM disparity 전처리 — Phase C+ deferred
- [ ] Epipolar 1D brute-force — Phase C+ (학습용)

## 가속 경로

- [x] CPU scalar (baseline)
- [ ] CPU SIMD / MT / WebGL / WebGPU — Phase C+ deferred

## 검증

- [x] 자동 수치 게이트 — 매칭율 100% (200/200), mean dx = -21.95 px, mean |dy| = 0.00 px (synthetic stereo)
- [x] 시각 수동 게이트 — 빨강 = 실패 점, 파랑 = 성공 점 매칭선
- [x] 원본 C++ 결과와의 diff — OpenCV 직접 호출이라 일치

## 학습 노트

LK는 본질적으로 시간축 광학 흐름(t → t+1)을 풀지만, KITTI 정류 스테레오에서는 좌→우 매칭이 "동시각 두 카메라의 horizontal disparity"라는 동등한 문제로 환원된다. 책 구현은 좌측 keypoint 좌표를 그대로 우측에서의 초기 추정치로 사용(즉 disparity ≈ 0 가정)하며, 이는 baseline이 작거나 멀리 있는 점에는 잘 맞지만 가까운 점은 수렴 라운드가 늘어난다.

본 Step의 `useInitialFlow` 토글은 좌측 좌표를 초기치로 쓸지(true) 아니면 좌측 좌표 자체에서 시작할지(false)를 선택한다. KITTI 05의 baseline = 0.537 m / typical depth = 5~50 m → typical disparity = 5~50 px. 충분한 윈도우(11+) + level 4 pyramid에서 수렴은 안정적이다.

`bind_features.trackLK`의 입출력은 모두 stride 3 `[x, y, score]` — Step 3 detect 출력을 그대로 입력으로 받을 수 있게 통일. score는 LK에서는 무시하고 결과의 status flag(0/1)는 별도 mask로 반환.

### 의도된 실패

- window size를 7로 줄이면 텍스처 부족 영역(예: 균일한 회색 패치)에서 LK가 발산해 매칭 실패율 ↑.
- max iterations를 10으로 줄이면 큰 disparity(가까운 점)는 수렴 못 함 — 본 Step 7의 `maxIter sweep` 차트와 같은 패턴.
- pyramid level을 1로 → 큰 disparity는 추적 불가(coarse-to-fine 효과 사라짐).
