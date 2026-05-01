# Step 05 — Triangulation (Linear SVD)

## 학습 목표

- [x] DLT 방식 선형 삼각화 — `[x·m3 − m1; y·m3 − m2] X = 0`
- [x] σ4/σ3 조건수로 해 품질을 판정하는 방법
- [x] Linear SVD vs Midpoint 알고리즘의 정확도 trade-off

## C++ 원본 매핑

- 파일: `ch13/include/myslam/algorithm.h::triangulation`
- 알고리즘: 각 관측마다 2 row를 쌓아 2N×4 행렬 A를 만들고 BDC-SVD 후 최소 특이값 벡터(V의 마지막 열)가 해
- **반환 로직 버그**: σ₄/σ₃ < 1e-2일 때 `true`(quality bad), 좋을 때 `false` 반환 — 원서의 알려진 버그. 본 Step은 학습용 토글로 두 polarity 모두 관찰 가능.

## UI

- ParamPanel: algorithm picker(Linear SVD / Midpoint) · σ4/σ3 quality threshold · invertedReturn toggle
- Input View: 좌/우 keypoint 매칭 + K_l/K_r/T_l/T_r
- Output View:
  - **Top-down (X, Z)** scatter plot — 녹색=accepted, 빨강=rejected
  - **Depth histogram** — 32-bin
  - mean depth + median metric + 통계
- VerifyGate(자동 5건): WASM 로드 · L/R/calib 로드 · accepted ≥ 30 · mean depth > 0 ∧ ≤ 100 m

## 알고리즘

- [x] Linear SVD (원본) — JacobiSVD<4×4> in `bind_triangulation.cpp`
- [x] Midpoint — 두 ray의 최단 distance 중점
- [ ] Optimal Triangulation (Hartley-Sturm) — Phase D+ deferred

## 가속 경로

- [x] CPU scalar (baseline) — 28 KB Eigen-only WASM
- [ ] CPU SIMD / MT / WebGPU — Phase D+ deferred

## 검증

- [x] 자동 수치 게이트 — 60-pt 합성 GT round-trip maxErr 3.96e-14 m
- [x] 시각 수동 게이트 — invertedReturn 토글 시 accepted 수가 N ↔ (total−N)으로 즉시 반전
- [x] 원본 C++ 결과와의 diff — 같은 입력에서 algorithm.h::triangulation과 수치 일치

## 학습 노트

DLT 삼각화는 동차 좌표계에서 가장 단순한 다중-뷰 reconstructor. 각 관측 `(x, y)`에서 projection matrix M의 row 1, 2, 3을 사용해 두 개의 선형 방정식을 만들고, N개의 관측에서 2N개의 row를 쌓아 SVD로 동차 해를 구한다.

**책의 반환 로직 버그**는 학습 가치가 높다 — `σ₄/σ₃ < 1e-2`(작다)는 4번째 특이값이 0에 가깝다는 뜻으로 좋은 해(rank-3에 가까움)임을 의미하지만, 책 구현은 그 케이스에 `return true`를 반환한다. caller의 `if (return)`을 따라가 보면 사실 그 결과를 채택하는 polarity로 코드가 일관돼 의도한 동작은 맞으나 코드와 주석이 어긋나는 형태. 본 Step의 `invertedReturn` 토글로 학습자가 두 해석을 모두 관찰할 수 있다 — fixed=N개 accepted, inverted=(total−N) accepted.

WASM 바이너리는 28 KB(Eigen-only) — Step 8(391 KB g2o), Step 11(439 KB g2o BA)와 비교해 매우 작다. SVD는 4×4 행렬에 대한 single solve라 Emscripten의 native O3 최적화만으로 충분.

### 의도된 실패

- σ4/σ3 quality threshold를 0.5처럼 매우 크게 → 거의 모든 점이 rejected.
- T_l/T_r을 동일하게(baseline = 0) → 두 ray가 평행해 SVD 해가 발산, NaN 또는 무한대 depth 발생.
