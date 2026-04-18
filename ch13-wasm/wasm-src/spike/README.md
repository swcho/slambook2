# Phase A+ WASM Spike — g2o PnP / BA 동작 검증

PLAN.md §9 **Phase A+ Gate**의 핵심 질문 — _“g2o + Eigen + sparse 선형 솔버가 Emscripten/WASM에서 빌드·링크·수렴하는가?”_ — 에 대한 **실증 스파이크**입니다. 이 디렉터리는 아래 두 바인딩과 각각의 수치 검증 스크립트로 구성됩니다.

| 파일 | 목적 | 링크 라이브러리 |
|------|------|----------------|
| [`bind_pnp_spike.cpp`](./bind_pnp_spike.cpp) | **Stage 1**: pose-only PnP (단항 엣지) | `core + stuff + solver_dense` |
| [`bind_ba_spike.cpp`](./bind_ba_spike.cpp) | **Stage 2**: full Bundle Adjustment (이항 엣지 + Schur) | `core + stuff + solver_eigen` |
| [`verify_pnp.mjs`](./verify_pnp.mjs) | Stage 1 gate 테스트 | — |
| [`verify_ba.mjs`](./verify_ba.mjs) | Stage 2 gate 테스트 | — |
| `verify_pnp_diag{,2}.mjs` | 스파이크 디버깅 과정 기록 (참고용) | — |

---

## 0. 공통 수학 기초

### 0.1 표기

| 기호 | 의미 |
|------|------|
| $P_w \in \mathbb{R}^3$ | **월드 좌표계**에서의 3D 점 (landmark) |
| $P_c \in \mathbb{R}^3$ | **카메라 좌표계**에서의 3D 점 |
| $T_{cw} \in SE(3)$ | 월드→카메라 변환, 즉 $P_c = T_{cw} \cdot P_w$ |
| $K \in \mathbb{R}^{3 \times 3}$ | 핀홀 내부 파라미터 행렬 |
| $(u, v)$ | 영상에 투영된 픽셀 좌표 |
| $\xi = [\rho;\; \phi] \in \mathbb{R}^6$ | $SE(3)$의 **tangent vector**, $\rho$는 translation tangent, $\phi$는 rotation tangent (axis-angle) |

### 0.2 핀홀 투영

$$
P_c = T_{cw} \cdot P_w,\quad
K P_c = \begin{bmatrix} \tilde{u} \\ \tilde{v} \\ \tilde{w} \end{bmatrix},\quad
\begin{bmatrix} u \\ v \end{bmatrix} = \frac{1}{\tilde{w}}\begin{bmatrix} \tilde{u} \\ \tilde{v} \end{bmatrix}
$$

$K$는

$$
K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
$$

이므로, $P_c = (X, Y, Z)$일 때

$$
u = \frac{f_x X}{Z} + c_x, \qquad v = \frac{f_y Y}{Z} + c_y
$$

### 0.3 $SE(3)$ left-multiplicative update

g2o의 최적화 루프는 매 반복에서 **증분** $\xi \in \mathbb{R}^6$를 풀고 정점에 적용합니다. 본 스파이크는 ch13 책과 동일한 **left 곱 convention**을 사용합니다:

$$
T' = \exp(\xi^{\wedge}) \cdot T
$$

여기서 $\xi^{\wedge}$는 $\mathbb{R}^6$를 $\mathfrak{se}(3)$의 $4 \times 4$ 행렬로 올리는 hat 연산자:

$$
\xi^{\wedge} =
\begin{bmatrix} \phi^{\wedge} & \rho \\ 0^\top & 0 \end{bmatrix} \in \mathbb{R}^{4 \times 4}, \qquad
\phi^{\wedge} = \begin{bmatrix} 0 & -\phi_3 & \phi_2 \\ \phi_3 & 0 & -\phi_1 \\ -\phi_2 & \phi_1 & 0 \end{bmatrix}
$$

**본 스파이크의 단순화**: Sophus 대신 `Eigen::Isometry3d`를 쓰기 위해 다음과 같이 근사합니다.

$$
dT = \begin{bmatrix} \exp(\phi^{\wedge}) & \rho \\ 0^\top & 1 \end{bmatrix}
$$

엄밀한 Sophus SE(3)는 translation에 $V(\phi) \cdot \rho$ ($V$는 $SE(3)$의 left Jacobian)를 씁니다. 둘은 **1차 근사에서 동일**하므로 Jacobian(xi=0에서 평가)도 동일합니다. LM이 수렴하면서 step size가 작아지면 둘의 차이는 사라집니다.

### 0.4 2D reprojection error와 Jacobian

관측값을 $z \in \mathbb{R}^2$, 예측값을 $\hat{z}(T) = \pi(KT P_w)$라 할 때, 본 스파이크의 에러는 **측정 − 예측** (ch13 convention):

$$
e(T) = z - \hat{z}(T)
$$

Left update에 대한 Jacobian $\partial e / \partial \xi$의 유도:

1. $\partial P_c / \partial \rho = I_{3 \times 3}$
2. $\partial P_c / \partial \phi = -[P_c]_\times$ (skew-symmetric)
3. $\partial \pi / \partial P_c = \begin{bmatrix} f_x / Z & 0 & -f_x X / Z^2 \\ 0 & f_y / Z & -f_y Y / Z^2 \end{bmatrix}$
4. 체인 룰 + $e = z - \hat{z}$의 부호 반전 후 정리하면 **2×6 블록**:

$$
\frac{\partial e}{\partial \xi} =
\begin{bmatrix}
  -f_x/Z & 0 & f_x X / Z^2 & f_x XY / Z^2 & -f_x - f_x X^2/Z^2 & f_x Y / Z \\
  0 & -f_y/Z & f_y Y / Z^2 & f_y + f_y Y^2/Z^2 & -f_y XY / Z^2 & -f_y X / Z
\end{bmatrix}
$$

이 식은 `ch13/include/myslam/g2o_types.h::EdgeProjectionPoseOnly::linearizeOplus()`의 결과와 **열 순서까지 정확히 일치**합니다.

---

## 1. `bind_pnp_spike.cpp` — Stage 1 (Pose-only PnP)

### 1.1 목표

월드 좌표 3D 점들과 그에 대응하는 2D 픽셀 관측이 주어졌을 때, 카메라 포즈 $T_{cw}$ 하나를 최적화해 재투영 오차를 최소화합니다. g2o의 가장 작은 optimization problem — _vertex 1개, edge N개, linear solver는 dense_ — 이므로 **g2o 기본 경로가 Emscripten에서 돌아가는가**를 최소 비용으로 검증합니다.

### 1.2 정점: `VertexPoseSE3`

```cpp
class VertexPoseSE3 : public g2o::BaseVertex<6, Eigen::Isometry3d>
```

- 상태 차원 **6** (SE(3) tangent), 저장 타입 `Eigen::Isometry3d` (4×4 동차 변환).
- `setToOriginImpl()`: 추정치를 Identity로 초기화.
- `oplusImpl(update)`: 위의 §0.3 단순화를 구현.
  1. `update`의 앞 3요소를 $\rho$ (translation tangent), 뒤 3요소를 $\phi$ (axis-angle rotation)로 읽음
  2. `Eigen::AngleAxisd`로 $R_{exp} = \exp(\phi^{\wedge})$ 계산
  3. `dT = (R_exp, \rho)`를 만든 뒤 `_estimate = dT * _estimate` (left 곱)

### 1.3 엣지: `EdgeReprojectionPoseOnly`

```cpp
class EdgeReprojectionPoseOnly
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPoseSE3>
```

- 측정 차원 **2** (픽셀 $(u, v)$), 연결 정점 1개 (pose).
- 생성자로 **해당 관측에 대응하는 월드 3D 점** `point_world_`와 내부 파라미터 `K_`를 주입 (landmark는 최적화 대상이 아니므로 정점이 아니라 상수).
- `computeError()`: §0.4의 $e = z - \hat{z}$ 계산.
- `linearizeOplus()`: §0.4의 2×6 Jacobian을 해석해로 직접 채움. Numerical Jacobian을 쓰지 않는 이유 = 속도 + LM convergence 안정성.

### 1.4 최적화 알고리즘 세팅

```cpp
BlockSolver<BlockSolverTraits<6, 3>> + LinearSolverDense + LevenbergMarquardt
```

- Block solver template argument `<6, 3>`: pose block 6×6, landmark block 3×3 (PnP에서는 landmark는 optimize 안 하지만 타입은 그대로).
- `LinearSolverDense`: Eigen `LLT`로 6×6 Hessian을 풀어 단일 pose update를 구함. PnP는 시스템이 작아서 sparse까지 필요 없음.
- `OptimizationAlgorithmLevenberg`: 비선형 LM. 초기 $\lambda$ 자동 설정.

### 1.5 Embind 인터페이스

```ts
solvePnP(input: PnPInput): PnPResult

interface PnPInput {
  points3d_flat: number[];    // 3N length, [x0, y0, z0, x1, y1, z1, ...]
  points2d_flat: number[];    // 2N length, [u0, v0, u1, v1, ...]
  K_row_major:   number[];    // 9 length, row-major 3×3
  init_pose6:    number[];    // 0 or 6 length, [tx, ty, tz, rx, ry, rz]
                              //   (rx,ry,rz는 axis-angle; 빈 배열이면 Identity)
  max_iters:     number;      // LM 최대 반복 (0이면 입력 그대로 반환)
}

interface PnPResult {
  Tcw_row_major: VectorDouble;  // 16 length, row-major 4×4
  iterations:    number;        // LM이 수행한 실제 반복 수
  final_chi2:    number;        // 최종 활성 총 오차 제곱합
  converged:     boolean;       // iterations > 0 (단순 플래그)
}
```

JS 측에서 `Tcw_row_major`는 `emscripten::register_vector<double>`로 등록된 `VectorDouble` 핸들이므로 `.get(i)` / `.size()`로 접근해야 합니다. `Array.prototype.slice`는 동작하지 않으니 주의.

### 1.6 빌드 + 실행

```bash
# Stage 1 빌드
bash wasm-src/build.sh baseline pnp_spike

# Node 수치 검증 (Gate 테스트)
node wasm-src/spike/verify_pnp.mjs
```

### 1.7 기대 결과 (2026-04-18 최초 통과 기록)

| 시나리오 | Init | 최종 chi² | rotErr | trErr |
|----------|------|-----------|--------|-------|
| Noiseless × 5 seeds, N=40 | GT ± 0.3 | 10⁻¹⁹ ~ 10⁻²⁵ | 0 ~ 2.5 × 10⁻¹⁶ rad | 8.3 × 10⁻¹⁷ ~ 6.6 × 10⁻¹⁴ |
| 1 px Gaussian × 3 seeds, N=80 | GT ± 0.3 | 1.5 × 10² ~ 1.8 × 10² | 6 × 10⁻⁴ ~ 1 × 10⁻³ rad | 5.7 × 10⁻⁴ ~ 1.1 × 10⁻³ |

→ 두 경우 모두 PASS. g2o core + Eigen + dense 선형 솔버가 WASM에서 동작한다는 강한 증거.

---

## 2. `bind_ba_spike.cpp` — Stage 2 (Bundle Adjustment)

### 2.1 목표

여러 카메라 포즈와 여러 landmark를 **동시에** 최적화하는 BA 문제를 g2o의 **이항 엣지 + `setMarginalized(true)` Schur complement 파이프라인**으로 푸는 것이 WASM에서 가능한지 검증합니다. 이는 실제 Phase G Step 11 Bundle Adjustment의 축소판입니다.

### 2.2 추가 정점: `VertexXYZ`

```cpp
class VertexXYZ : public g2o::BaseVertex<3, Eigen::Vector3d>
```

- 상태 차원 **3** (월드 좌표계의 3D 점), 저장 타입 `Vector3d`.
- `oplusImpl(update)`: 단순 덧셈 `_estimate += Map<Vector3d>(update)`.

### 2.3 이항 엣지: `EdgeProjection`

```cpp
class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPoseSE3, VertexXYZ>
```

- 측정 차원 **2**, 정점 2개 (pose, landmark).
- `computeError()`: `Pc = pose * Pw`로 landmark를 카메라 좌표로 옮긴 뒤 §0.2로 투영.
- `linearizeOplus()`: **두 개의 Jacobian**을 채워야 함.

#### Pose Jacobian ($2 \times 6$) — `_jacobianOplusXi`

단항 엣지와 동일한 식 (§0.4). 코드도 그대로 복붙.

#### Landmark Jacobian ($2 \times 3$) — `_jacobianOplusXj`

$P_w$에 대한 미분은 $P_c = R \cdot P_w + t$ 이므로 $\partial P_c / \partial P_w = R$.

$$
\frac{\partial e}{\partial P_w}
= \frac{\partial e}{\partial P_c} \cdot \frac{\partial P_c}{\partial P_w}
= \underbrace{\left[\frac{\partial e}{\partial \rho}\right]}_{\text{Xi의 앞 3열}}
  \cdot R
$$

왜 “`_jacobianOplusXi`의 앞 3열”이냐: 앞에서 유도한 대로 $\partial e / \partial \rho = \partial e / \partial P_c \cdot I$이므로, pose Jacobian의 translation block은 $\partial e / \partial P_c$와 **완전히 동일**합니다. 즉,

```cpp
_jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * T.linear();
```

— 이 한 줄이 ch13 원본(`EdgeProjection::linearizeOplus`)과 동일한 결과를 냅니다 (ch13은 `cam_ext.R * T.R`을 쓰지만 본 스파이크는 카메라 외부 파라미터가 없는 단일 카메라이므로 `T.R`만).

### 2.4 최적화 알고리즘 세팅

```cpp
BlockSolver<BlockSolverTraits<6, 3>> + LinearSolverEigen + LevenbergMarquardt
```

**Stage 1과의 차이**:

- `LinearSolverEigen` → `Eigen::SimplicialLLT<SparseMatrix, Upper>`를 내부에서 사용. **진짜 sparse Cholesky**. (확인: `grep -E "Simplicial|SparseMatrix" third_party/g2o/g2o/solvers/eigen/linear_solver_eigen.h`)
- `VertexXYZ::setMarginalized(true)`: 매 iteration에서 landmark block을 **Schur complement**로 소거해 작은 pose-only linear system으로 줄이는 g2o 관용구. BA가 대규모로 확장 가능해지는 핵심 트릭.
- 첫 번째 pose (id=0)는 `setFixed(true)` — **gauge freedom** 제거. 단, 모노큘러 세팅이라 **scale gauge는 여전히 자유** (모든 translation + landmark를 동일 스케일로 곱하면 projection invariant).

### 2.5 Embind 인터페이스

```ts
solveBA(input: BAInput): BAResult

interface BAInput {
  init_poses:     number[];  // 6 * numPoses 길이, pose 하나당 [tx, ty, tz, rx, ry, rz]
  init_landmarks: number[];  // 3 * numLandmarks 길이, [x, y, z, x, y, z, ...]
  observations:   number[];  // 4 * numObs 길이, obs 하나당 [poseIdx, landmarkIdx, u, v]
  K_row_major:    number[];  // 9 길이, row-major K
  fixed_poses:    number[];  // 고정시킬 pose index 배열 (gauge 제거)
  max_iters:      number;
}

interface BAResult {
  refined_poses:     VectorDouble;  // 12 * numPoses (row-major 3×4 per pose)
  refined_landmarks: VectorDouble;  // 3 * numLandmarks
  iterations:        number;
  initial_chi2:      number;        // optimize 호출 직전의 총 오차
  final_chi2:        number;
}
```

### 2.6 빌드 + 실행

```bash
bash wasm-src/build.sh baseline ba_spike
node wasm-src/spike/verify_ba.mjs
```

### 2.7 기대 결과 (2026-04-18 최초 통과 기록)

| 시나리오 | P (poses) | L (landmarks) | noise | chi² 감소 | 비고 |
|----------|-----------|---------------|-------|-----------|------|
| Noiseless × 3 seeds | 3 | 20 | 0 px | 1.97 × 10⁵ → 1.08 × 10⁻²² (~28 자릿수) | scale gauge drift ≤ 2.6 × 10⁻² |
| 1 px Gaussian × 3 seeds | 4 | 40 | 1 px | 1.28 × 10⁶ → 2.14 × 10² (~3.5 자릿수) | scale drift + 1 px noise 잔차 |

→ 두 경우 모두 PASS. sparse BA가 WASM에서 수렴.

**무노이즈에서 chi²가 10⁻²²까지 떨어졌는데도 `maxTr`가 10⁻³ 수준으로 남는 이유**: 모노큘러 BA의 **scale gauge 자유도**. 1개의 fixed pose만으로는 7개의 gauge DOF 중 6개만 고정됨 (scale은 미고정). 실제 ch13 파이프라인은 stereo baseline이 scale을 고정하므로 이 문제는 Phase G에서 발생하지 않습니다.

---

## 3. Phase A+ Gate 판정

Stage 1 + Stage 2 모두 통과 → **g2o 경로 채택 확정**.

- ✅ PLAN.md §9 Phase A+의 “성공” 분기 적용, **minimal LM 피벗 불필요**.
- ⚠️ **CXSparse 자체의 Emscripten 빌드는 미검증** — `solver_eigen`으로 치환됨. Phase G 성능 측정에서 SimplicialLLT가 active window 10+ keyframes 기준으로 부족하면 그 시점에 CXSparse 벤더링 시도 (추정 0.5 ~ 1일 추가 작업).

---

## 4. 재빌드가 필요한 경우

| 변경 | 명령 |
|------|------|
| 스파이크 C++ 수정 | `bash wasm-src/build.sh baseline pnp_spike` (또는 `ba_spike`) |
| g2o CMake 옵션 조정 | `rm -rf wasm-src/build/<target>_baseline` → 위 명령 |
| g2o submodule 업그레이드 | `git -C wasm-src/third_party/g2o fetch && git -C … checkout <sha>` |
| `verify_*.mjs` 수정 | 재빌드 불필요; 그냥 `node wasm-src/spike/verify_*.mjs` |

SIMD/MT variant 빌드도 CMakeLists의 `VARIANT_FLAGS` 분기로 바로 지원 — 예: `bash wasm-src/build.sh simd pnp_spike`. 다만 현재 gate 테스트는 baseline variant만 검증했으므로 Phase E 착수 시 정식 다중 variant 벤치 매트릭스를 짜게 됩니다.
