# Step 08 — Pose Estimation (PnP via g2o)

## 학습 목표

- [x] 비선형 최적화 + 아웃라이어 제거의 상호작용 — 책의 4-라운드 outlier 마킹 패턴
- [x] g2o의 VertexPose + EdgeProjectionPoseOnly 그래프 구성
- [x] Levenberg-Marquardt 수렴이 init 품질에 강하게 의존함을 확인

## C++ 원본 매핑

- 파일: `ch13/src/frontend.cpp::Frontend::EstimateCurrentPose` + `ch13/include/myslam/g2o_types.h::EdgeProjectionPoseOnly`
- 핵심: 4 × `optimizer.optimize(10)` 라운드, 매 라운드 후 `chi² > 5.991`인 엣지를 `setLevel(1)`로 비활성화, 라운드 ≥ 2부터 RobustKernel 제거.

## UI

- ParamPanel:
  - PnP algorithm picker (g2o LM = 활성, `cv::solvePnPRansac`/EPnP/DLS PnP = disabled chip)
  - rounds(1..4) · iterations per round(1..30) · chi² threshold · Huber δ · drop-after-round · useRobustKernel
- Input View: 3D-2D pair 통계 (Step 5/6/7 파이프라인 결과)
- Output View:
  - **라운드 슬라이더** — 라운드별 inlier mask + chi² 분포 애니메이션
  - **Pose 카드** — refined translation/rotation
  - **chi² histogram** — before/after × log scale
- VerifyGate(자동 4건): 3 WASM 로드 · prev+curr+calib · pair ≥ 30 · final chi² 유한 + inlier > 70%

## 알고리즘

- [x] g2o LM (원본) — Phase A+ Stage 1 spike 승격, `bind_pnp.cpp` (391 KB)
- [ ] cv::solvePnPRansac — Phase E+ deferred (calib3d 모듈 추가 빌드 필요)
- [ ] EPnP / DLS PnP — Phase E+ deferred

## 가속 경로

- [x] CPU scalar (baseline)
- [ ] CPU SIMD / MT / WebGPU — Phase C+ deferred

## 검증

- [x] 자동 수치 게이트 — `verify_pnp.ts` 14건 통과 (noiseless 5 + 1px 3 + 20% seeded outlier 3 + RobustKernel 토글 3)
- [x] 시각 수동 게이트 — 라운드 슬라이더로 outlier가 라운드를 거칠수록 빨강으로 마킹되어 줄어드는 모습 관찰
- [x] 원본 C++ 결과와의 diff — Phase A+ spike에서 ch13의 Vertex/Edge 그대로 사용해 수치 일치 확인

## 학습 노트

`VertexPoseSE3`는 SE(3)를 `Eigen::Isometry3d`로 표현하고 left-update(`T_new = exp(δ) · T_old`) 방식으로 갱신한다. Sophus 의존을 피하기 위한 선택 — Phase A+ 스파이크에서 검증됨. `EdgeReprojectionPoseOnly`의 Jacobian 2×6은 ch13 `g2o_types.h`와 동일한 해석해.

**4-라운드 outlier 루프는 단순한 RANSAC 대안이 아니라 LM 안정화 기법**. 첫 라운드는 RobustKernel(Huber δ=√5.991)을 켜두고 outlier를 부드럽게 줄여 수렴을 안정화시키고, 라운드 2부터는 kernel을 떼고 inlier 정밀도를 끌어올린다. 라운드별 inlier mask + chi² 평탄 배열을 노출해 UI에서 라운드 슬라이더 1개로 outlier 변화 애니메이션을 만들 수 있다 — 추가 C++ 호출 비용 0.

**init 품질 의존성**: Phase A+ 진단에서 GT 포즈로 init했을 때 chi²≈1.4e-26으로 수렴하지만 identity init은 수렴 실패. ch13의 `relative_motion_ * last_pose` init이 항상 좋은 prior를 제공하는 이유 — 본 Step은 Step 7 결과를 prior로 받아 그 의존성을 학습자가 직접 관찰.

`cv::solvePnPRansac`이 deferred인 이유 — calib3d 모듈을 BUILD_LIST에 추가해야 하고 OpenCV 분리 빌드 캐시가 무효화됨. EPnP/DLS도 calib3d 안에 있어 함께 처리(Phase E+).

### 의도된 실패

- Identity init (GT prior 무시) → 수렴 실패 + chi² 폭증.
- chi² threshold를 0.05처럼 매우 작게 + RobustKernel 끄기 → 적응적 doublings 4~5회 발생, threshold가 우측으로 이동.
- Huber δ를 매우 크게 → kernel 효과가 사실상 없어져 큰 outlier가 inlier로 취급됨.
