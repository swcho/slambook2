# Stage 1의 최적화 알고리즘 세팅은?

## 문제

ch13 visual SLAM에서 **Stage 1 (frontend pose-only PnP)**의 g2o 최적화 알고리즘은 어떻게 세팅되어 있는가?

## 짧은 답

`BlockSolver<BlockSolverTraits<6, 3>>` + `LinearSolverDense<PoseMatrixType>` + `OptimizationAlgorithmLevenberg` 조합을 사용하며, 내부적으로 Eigen LLT(Cholesky)로 **6×6 Hessian**을 푼다.

소스: `ch13/src/frontend.cpp::Frontend::EstimateCurrentPose` (L142–151).

```cpp
typedef g2o::BlockSolver_6_3                                        BlockSolverType;
typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>     LinearSolverType;

auto solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<BlockSolverType>(
        g2o::make_unique<LinearSolverType>()));

g2o::SparseOptimizer optimizer;
optimizer.setAlgorithm(solver);
```

---

## 상세 설명

### 1. 3-층 구조: g2o의 solver stack

g2o 옵티마이저는 **상속이 아니라 합성(composition)**으로 만든다. 안쪽에서 바깥쪽 순서로 세 컴포넌트를 끼워 넣는다.

```
┌────────────────────────────────────────────────────┐
│ OptimizationAlgorithmLevenberg  ← 비선형 알고리즘  │
│ ┌────────────────────────────────────────────────┐ │
│ │ BlockSolver_6_3                ← 블록 구조    │ │
│ │ ┌──────────────────────────────────────────┐  │ │
│ │ │ LinearSolverDense<PoseMatrixType>  ← Ax=b│  │ │
│ │ └──────────────────────────────────────────┘  │ │
│ └────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────┘
```

| 층 | 역할 | LM 한 스텝에서 하는 일 |
|----|------|------------------------|
| **OptimizationAlgorithmLevenberg** | 비선형 반복 (LM) | 댐핑 λ를 조절하며 $(H + \lambda I)\Delta x = -b$를 반복적으로 풂 |
| **BlockSolver_6_3** | 블록 구조 + Schur 보조 | residual로 $H, b$ 조립, pose-only면 그대로, BA면 landmark Schur complement 계산 |
| **LinearSolverDense** | 선형방정식 $A\Delta x = c$ | 매 스텝 Eigen LLT/LDLT로 푸는 가장 안쪽 핵 |

### 2. 왜 `BlockSolver_6_3`인가? — 차원 컨벤션

`BlockSolver_6_3`는 다음의 typedef.

```cpp
typedef BlockSolver< BlockSolverTraits<6, 3> > BlockSolver_6_3;
```

`BlockSolverTraits<PoseDim, LandmarkDim>`의 두 템플릿 인자는

- `PoseDim = 6` — SE(3) twist의 자유도 (Lie algebra $\xi \in \mathbb{R}^6$)
- `LandmarkDim = 3` — 3D point의 자유도

Stage 1에서는 **pose vertex 1개**만 등록하고 landmark는 모두 fixed로 들어가지 않으므로 실제로 활성화되는 블록은 6×6 한 개뿐이다 (`LandmarkDim = 3`은 같은 typedef를 backend BA가 공유하기 때문에 남아 있을 뿐 frontend에서는 작동하지 않는다).

> 책 7장의 BA용 typedef를 그대로 재사용하는 이유는 Edge Jacobian 블록 차원과 매칭이 맞기 때문 — `EdgeProjectionPoseOnly`의 Jacobian은 $2\times 6$, `EdgeProjection`(BA용)은 $2\times 6 + 2\times 3$.

### 3. 왜 `LinearSolverDense`인가? — 6×6은 dense가 최적

Linear solver 선택지는 보통 셋이다.

| Solver | 시간 복잡도 | 적합한 경우 | Frontend 사용? |
|--------|-------------|-------------|----------------|
| `LinearSolverDense` | $O(n^3)$ | $n$ 작음 (≤ 수백), Hessian이 fully dense | ✅ Stage 1 (n=6) |
| `LinearSolverCSparse` | sparse Cholesky | 중규모 sparse SLAM | backend.cpp |
| `LinearSolverCholmod` | suite-sparse Cholesky | 대규모 BA (수만 노드) | — |

Stage 1의 Hessian은 정확히 $6 \times 6$ — landmark가 모두 measurement 쪽으로 흡수된 단일 pose 그래프이기 때문에 sparsity 패턴이 없고 "조밀한 6×6 SPD"이다. CSparse/Cholmod는 sparsity bookkeeping 오버헤드만 추가될 뿐 이득이 없다. **dense LLT가 캐시 친화적이고 가장 빠르다.**

`LinearSolverDense<PoseMatrixType>`의 내부:

```cpp
// g2o/solvers/linear_solver_dense.h (요약)
Eigen::LLT<MatrixXd> llt(H);   // SPD Cholesky
x = llt.solve(b);
```

LM이 $H + \lambda I$ 로 댐핑하면 SPD가 항상 유지되므로 `LDLT`가 아닌 **`LLT`(strict positive definite Cholesky)**가 안전하게 쓰인다.

### 4. 왜 Levenberg-Marquardt인가? — Gauss-Newton과의 비교

g2o의 비선형 알고리즘은 셋:

| 알고리즘 | 업데이트 식 | 특징 | Stage 1 적합도 |
|----------|-------------|------|----------------|
| `OptimizationAlgorithmGaussNewton` | $H \Delta x = -b$ | init이 좋으면 가장 빠름, 발산 위험 | △ |
| `OptimizationAlgorithmLevenberg` | $(H + \lambda I)\Delta x = -b$ | trust-region, λ 적응 | ✅ |
| `OptimizationAlgorithmDogleg` | trust-region + line search | 더 robust, 구현 복잡 | — |

Pose-only PnP는 init이 $T_{rel} \cdot T_{last}$로부터 오기 때문에 **대체로 좋은 prior**가 있지만, 모션이 클 때(빠른 회전 등) GN이 발산할 수 있다. LM은 `λ`를 자동으로 조절해 GN ↔ steepest descent 사이를 미끄러지므로 **init 품질에 덜 민감**하다.

LM 한 스텝의 흐름:
1. 현재 $\xi$에서 $H, b$ 조립 (Edge가 chain rule로 Jacobian 계산)
2. Dense LLT로 $(H + \lambda I)\Delta\xi = -b$ 풀기
3. 시도 update $\xi' = \exp(\Delta\xi^\wedge)\,\xi$
4. cost가 줄면 채택, λ ÷ 10; 늘면 거부, λ × 10
5. 수렴 또는 max iter까지 반복

`optimizer.optimize(10)`은 위 1–5를 최대 10회 돌린다. Stage 1은 4-라운드 outlier 루프 안에서 매 라운드 10회씩 돌리므로 이론상 최대 40회의 LM 스텝.

### 5. Vertex / Edge와의 인터페이스

세팅된 solver가 실제로 다루는 변수는 graph에 등록된 vertex/edge로부터 온다.

```cpp
VertexPose *vertex_pose = new VertexPose();
vertex_pose->setId(0);
vertex_pose->setEstimate(current_frame_->Pose());
optimizer.addVertex(vertex_pose);

// for each map point with 2D observation
EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
edge->setVertex(0, vertex_pose);
edge->setMeasurement(toVec2(feat->position_.pt));
edge->setInformation(Eigen::Matrix2d::Identity());
edge->setRobustKernel(new g2o::RobustKernelHuber);
optimizer.addEdge(edge);
```

여기서 BlockSolver가 자동으로 다음을 해 준다.

- `VertexPose::oplusImpl(δ)`: $T \leftarrow \exp(\delta^\wedge)\,T$ — left perturbation
- `EdgeProjectionPoseOnly::linearizeOplus()`: $J = \partial e / \partial \xi \in \mathbb{R}^{2\times 6}$ 채우기
- 매 LM 스텝 $H = \sum J^\top \Omega J$, $b = \sum J^\top \Omega e$ 누적
- Robust kernel(Huber)이 켜져 있으면 가중치 자동 적용

### 6. Backend (Stage 2) 와의 차이 — 컨트라스트

같은 `BlockSolver_6_3`를 쓰지만 Stage 2(backend.cpp)는 다른 결정을 내린다.

| 항목 | Stage 1 (frontend) | Stage 2 (backend) |
|------|---------------------|-------------------|
| LinearSolver | **`LinearSolverDense`** | **`LinearSolverCSparse`** |
| Vertex | pose 1개 | pose N개 + landmark M개 |
| Hessian | 6×6 dense | (6N + 3M) × (6N + 3M) sparse |
| Schur complement | 사용 안 함 | landmark 마진화로 6N×6N으로 축소 |
| 알고리즘 | LM | LM |
| optimize() 횟수 | 라운드당 10, 4 라운드 | 10 (1회) |

Frontend는 **속도**(매 프레임 호출, latency 직접 영향)가, backend는 **확장성**(N, M이 커질수록 sparse가 필수)이 우선이라 같은 BlockSolver typedef 위에 다른 LinearSolver를 끼워 넣는 것이 g2o 합성 구조의 핵심 활용 패턴.

### 7. 4-라운드 루프와의 결합

Stage 1의 진짜 모양은 단순한 "한 번 LM"이 아니라 outlier rejection과 결합된 **adaptive LM**이다.

```cpp
for (int iteration = 0; iteration < 4; ++iteration) {
    vertex_pose->setEstimate(current_frame_->Pose());  // 매 라운드 init 리셋
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    // chi² > 5.991인 edge → setLevel(1) (다음 라운드에서 비활성)
    // iteration == 2 부터 RobustKernel 제거
}
```

- **라운드 0–1**: Huber kernel ON, 큰 outlier에 의한 quadratic blow-up을 흡수해 LM이 안정 수렴.
- **라운드 2–3**: kernel OFF, inlier만 남은 상태에서 pure least-squares로 정밀도 끌어올림.
- 매 라운드 init을 `current_frame_->Pose()`로 다시 세팅 → outlier가 잘못 끌어당긴 estimate를 매 라운드 시작점으로 끌어오는 효과.

따라서 Stage 1의 옵티마이저 세팅은 **(LM + Dense Cholesky)** 라는 알고리즘 한 줄이 아니라, 그 위에 얹힌 **chi² gating + RobustKernel toggling**까지 한 묶음으로 설계된 것이다.

### 8. 한눈에 보는 요약

| 컴포넌트 | 클래스 / 함수 | 의미 |
|----------|---------------|------|
| 비선형 알고리즘 | `OptimizationAlgorithmLevenberg` | LM, λ 자동 조정 |
| 블록 구조 | `BlockSolver_6_3` = `BlockSolver<BlockSolverTraits<6,3>>` | pose 6, landmark 3 (Stage 1은 pose만 활성) |
| 선형 솔버 | `LinearSolverDense<PoseMatrixType>` | Eigen LLT, 6×6 dense Cholesky |
| 그래프 | `SparseOptimizer` | vertex/edge 등록, optimize() 호출 |
| Vertex | `VertexPose` (custom) | SE(3), left-perturbation update |
| Edge | `EdgeProjectionPoseOnly` (custom) | 2×6 Jacobian, reprojection residual |
| 외부 루프 | 4-round chi² gating + Huber toggle | adaptive outlier rejection |

## 참고

- `ch13/src/frontend.cpp` (L142–151, L187–212)
- `ch13/include/myslam/g2o_types.h` — `VertexPose`, `EdgeProjectionPoseOnly`
- `ch13/src/backend.cpp` (L45–50) — 같은 BlockSolver 위에 CSparse를 끼운 비교 대상
- g2o `core/optimization_algorithm_levenberg.h`, `solvers/dense/linear_solver_dense.h`
- Gao Xiang, *Visual SLAM 14강* (2판) §9.2 — Frontend optimization 구조
- 본 저장소: [`docs/steps/08-pose-estimation.md`](./steps/08-pose-estimation.md), [`docs/qa-pose-only-jacobian.md`](./qa-pose-only-jacobian.md)
