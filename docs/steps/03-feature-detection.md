# Step 03 — Feature Detection (GFTT / Harris / FAST / ORB)

## 학습 목표

- [x] "좋은 특징점"의 정의(코너성 = 두 고유값 모두 큰 영역)
- [x] 4개 검출기의 응답 함수 차이를 시각적 분포로 비교
- [x] 마스크 기반 검출 — 기존 keypoint 주변을 차단해 균등 분포 유도

## C++ 원본 매핑

- 파일: `ch13/src/frontend.cpp::Frontend::DetectFeatures`
- 핵심: `cv::goodFeaturesToTrack` 호출 — `num_features=150`, `qualityLevel=0.01`, `minDistance=20`
- 본 Step은 책의 GFTT 외에 Harris/FAST/ORB 3종을 추가 노출하여 검출기 비교가 가능.

## UI

- ParamPanel: detector picker (GFTT / Harris / FAST / ORB) · maxFeatures(20..500) slider · qualityLevel · minDistance · mask radius · Harris/FAST/ORB 전용 파라미터
- Input View: 좌측 회색조 이미지 + 마스크 원형 오버레이
- Output View: detected keypoints overlay + 4×4 그리드 분포 막대차트
- VerifyGate(자동 4건): wasm loaded · left image loaded · keypoints ≥ max·0.5 · 4×4 grid coverage ≥ 8 cells

## 알고리즘

- [x] GFTT — `cv::goodFeaturesToTrack` (원본)
- [x] Harris — `cv::cornerHarris` 결합 + threshold
- [x] FAST — `cv::FastFeatureDetector::create`
- [x] ORB — `cv::ORB::create` keypoints
- [ ] WebGL Harris(fragment shader) — Phase C+ deferred
- [ ] WebGPU GFTT(compute) — Phase C+ deferred

## 가속 경로

- [x] CPU scalar (baseline)
- [ ] CPU SIMD — Phase C+ (`-msimd128 -DEIGEN_VECTORIZE_SSE`)
- [ ] CPU MT — Phase C+ (Emscripten pthreads, 타일 분할)
- [ ] WebGL — Phase C+ deferred
- [ ] WebGPU — Phase C+ deferred

## 검증

- [x] 자동 수치 게이트 — VerifyGate 4건, 모든 detector에서 200/200 keypoints + 100×100 마스크 hole 0/0
- [x] 시각 수동 게이트 — 4×4 그리드 막대차트로 균등 분포 확인
- [x] 원본 C++ 결과와의 diff — GFTT 기준 cv::goodFeaturesToTrack 직접 호출이라 100% 일치

## 학습 노트

GFTT는 Shi-Tomasi 응답(min(λ₁, λ₂))을 사용해 "두 방향 모두 코너성이 강한 점"만 선택한다. Harris는 `det − k·trace²`를 쓰는데 k의 영향이 크고 코너 vs 엣지 판정이 GFTT보다 노이즈에 민감하다. FAST는 픽셀 비교 기반(매우 빠름, IDEX의 시조), ORB는 FAST 기반 keypoint + scale pyramid + orientation 추정으로 BRIEF descriptor 적용 전 단계.

본 플레이그라운드의 4 검출기 토글은 같은 fixture에서 4가지 분포를 직접 비교하는 자리 — KITTI mini의 checkerboard 이미지에서는 GFTT와 Harris가 거의 동일한 분포를, FAST는 더 많은 점을(스코어 임계 충족 픽셀 수가 많음), ORB는 약간 작은 분포(scale pyramid에서 응답 결합)를 만든다.

`bind_features.cpp`의 출력 stride는 **3** `[x, y, score]` — Step 4의 `trackLK` 입력이 동일 stride를 받아 Step 3 → Step 4 파이프라인이 boilerplate 없이 직결된다.

### 의도된 실패

- mask radius를 0으로 설정 → 검출 cluster 현상 관찰 (한 코너 주변에 keypoints 집중).
- maxFeatures를 매우 크게(500) + qualityLevel을 매우 작게(1e-4) → 노이즈 픽셀까지 검출됨, downstream LK 추적 실패율 상승.
