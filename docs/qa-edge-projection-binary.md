# `EdgeProjection` 이항 엣지의 연결 정점과 측정 차원

## 문제

`EdgeProjection` 이항 엣지의 연결 정점과 측정 차원은?

## 짧은 답

**측정 차원 2**, **연결 정점 2개**(`VertexPoseSE3` pose와 `VertexXYZ` landmark)이다. 선언은 다음과 같다.

```cpp
class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPoseSE3, VertexXYZ> {
  ...
};
```

---

## 상세 설명

### 1. `BaseBinaryEdge` 템플릿 시그니처

g2o의 이항 엣지 베이스 클래스는 다음 4개의 템플릿 인자를 받는다.

```cpp
template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseBinaryEdge;
```

| 인자 | 의미 | `EdgeProjection`의 값 |
|------|------|------------------------|
| `D`  | 측정값/오차의 차원 | `2` (픽셀 평면의 reprojection error) |
| `E`  | 측정값 타입       | `Eigen::Vector2d` (관측 픽셀 $(u, v)$) |
| `VertexXi` | 첫 번째 정점 타입 (`_vertices[0]`) | `VertexPoseSE3` (카메라 pose) |
| `VertexXj` | 두 번째 정점 타입 (`_vertices[1]`) | `VertexXYZ` (3D landmark) |

따라서 측정/오차 벡터는 $\mathbb{R}^2$, 연결 정점은 정확히 2개다 (이항 엣지의 정의 자체가 "정점 2개를 잇는 엣지"이며, BA의 reprojection factor가 그 표준 예시이다).

### 2. 연결 정점의 역할

| 정점 | 차원 | 추정 변수 | 업데이트 방식 |
|------|------|-----------|----------------|
| `VertexPoseSE3` | 6 | $T \in SE(3)$, 내부 저장은 `Eigen::Isometry3d` | `oplusImpl`: $T \leftarrow \exp(\hat{\xi})\, T$ (left-update, $\xi = (\rho, \phi)$) |
| `VertexXYZ`     | 3 | $P_w \in \mathbb{R}^3$ (월드 좌표 점)            | `oplusImpl`: $P_w \leftarrow P_w + \delta$ (단순 덧셈) |

`_vertices[0]`이 pose, `_vertices[1]`이 landmark라는 순서는 `linearizeOplus()`의 자코비안 슬롯과 직결된다.

- `_jacobianOplusXi` ↔ pose 자코비안, $2 \times 6$
- `_jacobianOplusXj` ↔ landmark 자코비안, $2 \times 3$

### 3. 측정 차원이 2인 이유

오차 모델은 핀홀 reprojection이다.

$$
P_c = T_{\text{cam\_ext}} \, T \, P_w, \qquad
\hat{z} = \pi(P_c) = \begin{pmatrix} f_x X / Z + c_x \\ f_y Y / Z + c_y \end{pmatrix}, \qquad
e = z - \hat{z} \in \mathbb{R}^2.
$$

관측 $z$가 픽셀 좌표 $(u, v)$ 한 쌍이므로 오차 차원은 2가 된다. 이에 따라 정보행렬 $\Omega$도 $2 \times 2$이며, 픽셀 노이즈 분산이 등방적이면 $\Omega = I_2$로 둔다.

### 4. 코드 발췌 (ch13 / ch13-wasm)

`ch13/include/myslam/g2o_types.h` (정점 이름은 `VertexPose`이지만 같은 SE(3) pose):

```cpp
class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ> {
  ...
  void computeError() override {
    const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
    const VertexXYZ  *v1 = static_cast<VertexXYZ  *>(_vertices[1]);
    SE3 T = v0->estimate();
    Vec3 pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
  }
};
```

`ch13-wasm/wasm-src/myslam/bindings/bind_ba.cpp` (브라우저 빌드, `Eigen::Isometry3d` 기반의 `VertexPoseSE3` 사용):

```cpp
class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPoseSE3, VertexXYZ> {
  ...
  void computeError() override {
    const auto* vp = static_cast<const VertexPoseSE3*>(_vertices[0]);
    const auto* vx = static_cast<const VertexXYZ*>(_vertices[1]);
    Eigen::Vector3d Pc = cam_ext_ * (vp->estimate() * vx->estimate());
    Eigen::Vector3d uvw = K_ * Pc;
    _error = _measurement - Eigen::Vector2d(uvw.x() / uvw.z(),
                                            uvw.y() / uvw.z());
  }
};
```

두 경우 모두 시그니처는 `BaseBinaryEdge<2, Vector2d, <pose vertex>, VertexXYZ>`로 동일하며, 차이는 pose 정점이 Sophus `SE3`를 쓰느냐 `Eigen::Isometry3d`를 쓰느냐뿐이다 (참고: `qa-sophus-vs-isometry3d.md`).

### 5. Unary 버전과의 비교

같은 헤더의 `EdgeProjectionPoseOnly`는 landmark를 고정 상수로 두고 pose만 추정하므로 **단항 엣지**이다.

```cpp
class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose> { ... };
```

| 엣지 | 베이스 | 정점 수 | 측정 차원 | 자코비안 슬롯 |
|------|--------|---------|-----------|----------------|
| `EdgeProjectionPoseOnly` | `BaseUnaryEdge`  | 1 (pose)            | 2 | `_jacobianOplusXi` ($2\times 6$) |
| `EdgeProjection`         | `BaseBinaryEdge` | 2 (pose, landmark)  | 2 | `_jacobianOplusXi` ($2\times 6$), `_jacobianOplusXj` ($2\times 3$) |

즉 측정 차원 2는 동일하고, 연결 정점 수만 1 → 2로 늘어난다. 이것이 **PnP (pose-only)** 와 **BA (full bundle adjustment)** 의 그래프 구조 차이다.

---

## 한 줄 요약

`EdgeProjection`은 `BaseBinaryEdge<2, Vector2d, VertexPoseSE3, VertexXYZ>`를 상속하므로 — **측정 차원 = 2**, **연결 정점 = 2개**(pose + landmark)다.
