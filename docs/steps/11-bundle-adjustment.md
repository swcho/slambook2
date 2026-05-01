# Step 11 — Bundle Adjustment (Backend)

## 학습 목표

- [x] 이항 엣지(pose + landmark)와 Schur 보완으로 sparse BA 푸는 법
- [x] 적응적 chi² 임계 — 책의 `while(iteration<5)` 루프 의미
- [x] 좌/우 카메라 외부 파라미터 분기로 stereo BA 구현

## C++ 원본 매핑

- 파일: `ch13/src/backend.cpp::Backend::Optimize` + `ch13/include/myslam/g2o_types.h::EdgeProjection`
- 핵심: g2o `BlockSolver<6,3>` + `LinearSolverEigen`(SimplicialLLT, sparse Cholesky), VertexXYZ에 `setMarginalized(true)` → Schur 자동 활성화.

## UI

- ParamPanel:
  - active window toggle (2 KFs / 3 KFs) — mini fixture라 3..15는 Phase H로
  - iterations(1..30) · chi²_init(0.5..20) · adaptive rounds(0..8) · Huber δ · useRobustKernel · maxFeatures
  - solver: g2o + solver_eigen(active) / g2o + CXSparse(disabled chip) / direct LM(disabled chip)
- Input View: 4-WASM 파이프라인(features + triangulation + pnp + ba) 단일 화면, KF 0(앵커) + KF B/C 인덱스
- Output View:
  - **요약 카드** — observations / poses / iters / adaptive doublings / chi² init→final / drop ratio / build/BA ms
  - **Pose 표** — 각 pose의 refined translation + ‖Δt‖ + Δrot°
  - **chi² histogram (before vs after)** — 32-bin log scale, 회색=before, 파랑=after, 빨강=적응적 chi² 임계 점선
  - **Scene3D pre/post 토글** — 좌/우 frustum + landmark 클라우드(노랑=inlier, 빨강=outlier)
- VerifyGate(자동 5건): 4 WASM · 5 frames+calib · observations ≥ 60 · chi² 감소 ≥ 30%(완화) · 단조 수렴 + 유한값

## 알고리즘

- [x] g2o + solver_eigen(SimplicialLLT) + Schur (원본) — `bind_ba.cpp` (439 KB), Phase A+ Stage 2 spike 승격
- [ ] g2o + CXSparse — Phase G+ deferred (성능 측정 후 재평가)
- [ ] direct minimal LM — Phase I+ (학습용)

## 가속 경로

- [x] CPU scalar (baseline)
- [ ] CPU SIMD / MT(pthreads) / WebGPU — Phase H+ (active window 5+ KFs 측정 후 도입 결정)

## 검증

- [x] 자동 수치 게이트 — `verify_ba.ts` 10건 통과 (Stage 2 회귀 + 적응적 chi² + stereo cam_ext)
- [x] 시각 수동 게이트 — iterations 1 → 20에서 chi² histogram이 빨강 점선 왼쪽으로 모이는 수렴 시각화
- [x] 원본 C++ 결과와의 diff — 좌/우 cam_ext 분기 + 적응적 chi² 루프가 ch13 backend.cpp::Optimize와 동일

## 학습 노트

Bundle Adjustment는 SLAM의 "공식 정확도"를 결정한다. PnP(Step 8)가 단일 frame 포즈만 풀 때, BA는 active window 안의 모든 KF 포즈 + landmark를 동시에 풀어 전역적 일관성을 만든다.

**이항 엣지(BinaryEdge)** — `EdgeProjection`은 두 vertex(VertexPose + VertexXYZ)를 연결해 이 둘을 함께 최적화한다. 이게 PnP의 unary edge(VertexPose만)와 핵심 차이.

**Schur 보완**: BA의 normal equation이 6×N(pose) + 3×M(landmark)일 때, M이 N보다 훨씬 크다(KITTI에서 N=3~15, M=수백). VertexXYZ에 `setMarginalized(true)`를 두면 g2o가 자동으로 Schur 보완을 적용해 landmark 차원을 미리 소거하고 6×N pose 시스템만 풀고 나서 backsubstitution으로 landmark를 복원 — 이게 sparse BA가 빠른 핵심.

**적응적 chi² 루프**: 책의 `while(iteration<5)` 코드는 LM 추가 반복이 아니라 **임계값 후처리** — inlier 비율이 50% 미만이면 chi² 임계를 2배씩 늘려 더 많은 엣지를 살린다. 본 Step의 `adaptiveRounds` 토글이 그 동작을 그대로 재현.

**좌/우 cam_ext 분기**: spike에서는 monocular라 fixed_pose만으로 scale gauge를 제거할 수 없어 maxLm ~ 수 cm 드리프트가 발생했지만, ch13처럼 stereo baseline이 들어오면 그 자체가 scale을 고정해 maxLm = 7e-12까지 떨어진다. observation flat stride 5 `[poseIdx, lmIdx, u, v, isLeft]`로 좌/우 분기를 노출했다.

WASM 439 KB(spike 416 KB 대비 +5%) — 적응적 루프 + per-edge 메타 + 좌/우 ext 분기 코드만 추가, g2o core/stuff/solver_eigen 본체는 동일.

### 의도된 실패

- chi²_init = 0.05 + RobustKernel = false → 적응적 doublings 4~5회, 최종 임계 0.8까지 이동.
- iterations = 1 → 1회 LM step만, chi² 충분히 감소하지 못해 게이트 실패.
- active window = 2 KFs로 줄이고 iterations = 30 → KF 1개에 대한 over-fit, BA 효과 미미.
