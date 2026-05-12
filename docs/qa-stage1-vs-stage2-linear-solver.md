# Stage 2와 Stage 1의 선형 솔버 차이점은?

## 문제

ch13 visual SLAM에서 **Stage 1 (frontend pose-only PnP)**과 **Stage 2 (backend local BA)**가 g2o 선형 솔버로 서로 다른 구현체를 사용한다. 무엇이 어떻게 다른가?

## 짧은 답

| 단계 | LinearSolver | 내부 구현 | 행렬 모양 |
|------|--------------|-----------|-----------|
| Stage 1 (frontend) | `g2o::LinearSolverDense<PoseMatrixType>` | Eigen LLT (dense Cholesky) | 6×6 dense |
| Stage 2 (backend)  | `g2o::LinearSolverCSparse<PoseMatrixType>` | CSparse sparse Cholesky (`cs_chol` / `cs_cholsol`) | 6N×6N sparse (Schur complement 후) |

> 책 본문이나 일부 자료에서는 Stage 2에 `LinearSolverEigen<...>` (Eigen `SimplicialLLT<SparseMatrix, Upper>`) 를 쓴다고 설명하기도 한다. 둘 다 "sparse Cholesky"라는 점은 같지만 **백엔드 라이브러리가 다르다** — 본 저장소는 CSparse 버전을 채택하고 있다.

소스:
- Stage 1: `ch13/src/frontend.cpp::Frontend::EstimateCurrentPose` (L144–L150)
- Stage 2: `ch13/src/backend.cpp::Backend::Optimize` (L45–L52)

```cpp
// Stage 1 — frontend.cpp
typedef g2o::BlockSolver_6_3 BlockSolverType;
typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
    LinearSolverType;
auto solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<BlockSolverType>(
        g2o::make_unique<LinearSolverType>()));
```

```cpp
// Stage 2 — backend.cpp
typedef g2o::BlockSolver_6_3 BlockSolverType;
typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
    LinearSolverType;
auto solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<BlockSolverType>(
        g2o::make_unique<LinearSolverType>()));
```

`BlockSolver_6_3`(=`BlockSolverPL<6,3>`)와 `OptimizationAlgorithmLevenberg`는 동일하다. **다른 것은 `LinearSolver` 타입 단 한 줄**이다.

---

## 상세 설명

### 1. 왜 다른 솔버를 쓰는가 — 문제 크기와 희소성

**Stage 1 (Frontend, pose-only PnP)**
- 정점: `VertexPose` 1개 (6 DoF)
- 엣지: 현재 프레임의 inlier 관측치 N개 (단항 edge, 랜드마크는 fixed)
- 정규방정식: `H Δξ = -b`, 여기서 `H ∈ ℝ^{6×6}`
- **항상 6×6 dense 행렬** → sparse 자료구조의 오버헤드(symbolic factorization, AMD ordering 등)가 오히려 손해
- → **`LinearSolverDense`** + Eigen `LLT` (dense Cholesky) 가 최적

**Stage 2 (Backend, local BA)**
- 정점: 활성 키프레임 K개의 `VertexPose` (각 6 DoF) + 활성 랜드마크 M개의 `VertexXYZ` (각 3 DoF), `setMarginalized(true)`
- 엣지: 활성 키프레임에서 활성 랜드마크를 본 모든 양항 reprojection edge
- 랜드마크가 marginalize되므로 g2o가 **Schur complement** 로 reduced system을 만든다: `H_pose_reduced ∈ ℝ^{6K×6K}`
- 키프레임 i와 j가 같은 랜드마크를 공유할 때만 `H_pose_reduced` 의 (i,j) 블록이 채워짐 → **co-visibility 그래프 구조의 sparse SPD 행렬**
- → 큰 sparse system이므로 **sparse Cholesky** 필요 → `LinearSolverCSparse` (혹은 `LinearSolverEigen`/`LinearSolverCholmod`)

### 2. 두 LinearSolver 백엔드의 동작 차이

| 항목 | `LinearSolverDense` | `LinearSolverCSparse` |
|------|---------------------|------------------------|
| 헤더 | `g2o/solvers/dense/linear_solver_dense.h` | `g2o/solvers/csparse/linear_solver_csparse.h` |
| 자료구조 | `Eigen::Matrix<...>` (dense column-major) | CSparse `cs*` (CSC, compressed sparse column) |
| Cholesky | `Eigen::LLT` (in-place dense `LL^T`) | `cs_schol` (symbolic) + `cs_chol` (numeric) → `cs_ipvec`/`cs_lsolve`/`cs_ltsolve` |
| Ordering | 없음 (작아서 불필요) | AMD reordering으로 fill-in 최소화 |
| 비용 | `O(n^3)`, n=6 → 무시할 수준 | `O(nnz(L))`, fill-in에 비례 |
| 의존성 | Eigen 헤더만 | libcxsparse (CSparse 라이브러리) |

핵심: **둘 다 SPD 행렬에 대한 Cholesky 분해**를 수행하지만, "정규방정식 H의 모양과 크기"가 다르므로 자료구조와 알고리즘이 다르다.

### 3. `LinearSolverCSparse` vs `LinearSolverEigen` — 같은 sparse Cholesky인데 왜 헷갈리는가?

g2o는 동일한 sparse Cholesky 인터페이스에 대해 여러 백엔드를 제공한다:

| Linear solver | 라이브러리 | 알고리즘 |
|---------------|-----------|---------|
| `LinearSolverCSparse` | CSparse (Tim Davis) | `cs_chol` |
| `LinearSolverEigen` | Eigen | `Eigen::SimplicialLLT<SparseMatrix, Upper>` |
| `LinearSolverCholmod` | SuiteSparse CHOLMOD | supernodal sparse Cholesky |

세 가지 다 **희소 SPD에 대한 좌측 Cholesky 분해**라는 점에서 수학적으로 동치다. 다만:
- **`LinearSolverEigen`** 은 헤더-온리 Eigen만 쓰면 되어서 의존성이 가장 가벼움
- **`LinearSolverCSparse`** 는 libcxsparse 링크가 필요하지만 작고 빠름
- **`LinearSolverCholmod`** 는 가장 빠르지만 SuiteSparse 전체 의존성을 끌어옴

본 저장소는 빌드 시스템에 CSparse가 잡혀 있어 `LinearSolverCSparse` 를 쓴다.

### 4. 그래서 Stage 1 ↔ Stage 2의 본질적 차이

1. **Hessian이 dense이냐 sparse이냐**
   - Stage 1: `H` 가 6×6 → dense 자료구조가 정답
   - Stage 2: Schur complement 후에도 6K×6K 의 sparse 패턴 → sparse 자료구조가 정답

2. **Cholesky의 종류**
   - Stage 1: dense `LLT` (`Eigen::LLT`)
   - Stage 2: sparse `LL^T` (CSparse `cs_chol`, AMD reordering 포함)

3. **랜드마크 정점의 marginalize 여부**
   - Stage 1: 랜드마크가 정점이 아예 없음 (fixed) → Schur complement 자체가 발생하지 않음
   - Stage 2: 랜드마크 정점이 `setMarginalized(true)` → g2o block solver가 자동으로 Schur complement → reduced pose system을 sparse Cholesky로 푼다

4. **계산 비용 모델**
   - Stage 1: 매 LM step당 `O(6^3) = O(216)` — 사실상 상수
   - Stage 2: 매 LM step당 대략 `O(K · avg_covisible_neighbors · ...)` + Schur 보완 비용. 활성 윈도우의 keyframe 수 K, co-visibility 정도, fill-in에 좌우됨

---

## 요약

- **같은 점**: `BlockSolver_6_3`, `OptimizationAlgorithmLevenberg`, robust kernel 등 상위 layer는 동일.
- **다른 점**: 단 한 줄 — **`LinearSolverDense` (Stage 1)** ↔ **`LinearSolverCSparse` (Stage 2)**.
- **이유**: Stage 1은 6×6 dense 정규방정식, Stage 2는 Schur complement 후 6K×6K sparse SPD 시스템이기 때문이다. 문제의 구조에 자료구조를 맞추는 것.
