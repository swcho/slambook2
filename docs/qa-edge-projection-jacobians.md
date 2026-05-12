# `EdgeProjection::linearizeOplus`에서 채워야 하는 두 자코비안

## 문제

`EdgeProjection::linearizeOplus`에서 채워야 하는 두 Jacobian은?

## 짧은 답

pose에 대한 **`_jacobianOplusXi`** ($2\times 6$)와 landmark에 대한 **`_jacobianOplusXj`** ($2\times 3$) — 두 개다.

```cpp
// 1) Pose Jacobian: ∂e / ∂ξ ∈ ℝ^{2×6}
_jacobianOplusXi << ... ;

// 2) Landmark Jacobian: ∂e / ∂P_w ∈ ℝ^{2×3}
_jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0)
                 * _cam_ext.rotationMatrix() * T.rotationMatrix();
```

소스: `ch13/include/myslam/g2o_types.h::EdgeProjection::linearizeOplus` (L123–L143), `ch13-wasm/wasm-src/myslam/bindings/bind_ba.cpp::EdgeProjection::linearizeOplus` (L118–L145).

---

## 상세 설명

### 1. 두 자코비안은 어디서 오는가? — `BaseBinaryEdge` 슬롯

`BaseBinaryEdge<D, E, VertexXi, VertexXj>` 베이스 클래스는 두 정점 각각에 대응하는 자코비안 멤버를 미리 선언해 둔다.

| 멤버 | 의미 | 차원 (`EdgeProjection`) |
|------|------|--------------------------|
| `_jacobianOplusXi` | `_vertices[0]` (= pose) 에 대한 $\partial e / \partial \delta\xi$ | $2 \times 6$ |
| `_jacobianOplusXj` | `_vertices[1]` (= landmark) 에 대한 $\partial e / \partial \delta P_w$ | $2 \times 3$ |

`linearizeOplus()`의 책임은 단 하나 — **현재 추정값에서의 Jacobian 두 블록을 채워 넣는 것**이다. (g2o가 비워두면 numerical Jacobian으로 대체하지만 느리고 부정확하므로 reprojection 같은 잘 알려진 모델은 항상 analytic으로 채운다.)

### 2. 오차 모델과 chain rule

오차는 핀홀 reprojection이다.

$$
P_c = T_{\text{cam\_ext}} \, T \, P_w, \qquad
\hat{z} = \pi(P_c) = \begin{pmatrix} f_x X / Z + c_x \\ f_y Y / Z + c_y \end{pmatrix}, \qquad
e = z - \hat{z} \in \mathbb{R}^2.
$$

자코비안은 chain rule로 분해된다.

$$
\frac{\partial e}{\partial \delta\xi} = \underbrace{\frac{\partial e}{\partial P_c}}_{2\times 3}
\cdot \underbrace{\frac{\partial P_c}{\partial \delta\xi}}_{3\times 6}, \qquad
\frac{\partial e}{\partial \delta P_w} = \underbrace{\frac{\partial e}{\partial P_c}}_{2\times 3}
\cdot \underbrace{\frac{\partial P_c}{\partial \delta P_w}}_{3\times 3}.
$$

- $\partial e / \partial P_c$ — 핀홀 투영 미분 ($2\times 3$, fx/fy/Z 항). pose 자코비안과 landmark 자코비안이 **공유하는 부분**.
- $\partial P_c / \partial \delta\xi = [\,I_3 \;\;\; -P_c^\wedge\,]$ — SE(3) left perturbation의 Jacobian ($3\times 6$).
- $\partial P_c / \partial \delta P_w = R_{\text{cam\_ext}} \cdot R_T$ — extrinsic과 pose의 회전 부분만 남음 ($3\times 3$).

부호는 `_error = z − ẑ`이므로 두 자코비안 모두 통상 공식의 **음수**가 된다.

### 3. `_jacobianOplusXi` (Pose Jacobian, $2\times 6$)

좌측 perturbation $T \leftarrow \exp(\delta\xi^\wedge) T$ 컨벤션에서 $\delta\xi = (\rho, \phi) \in \mathbb{R}^6$ ($\rho$ = translation, $\phi$ = rotation 순서, 책 4장과 g2o의 SE(3) 규약).

해석해를 정리하면 Visual SLAM 14강 식 (7.46) 형태가 된다.

$$
\frac{\partial e}{\partial \delta\xi} =
-\begin{bmatrix}
\dfrac{f_x}{Z} & 0 & -\dfrac{f_x X}{Z^2} & -\dfrac{f_x X Y}{Z^2} & f_x + \dfrac{f_x X^2}{Z^2} & -\dfrac{f_x Y}{Z} \\[6pt]
0 & \dfrac{f_y}{Z} & -\dfrac{f_y Y}{Z^2} & -f_y - \dfrac{f_y Y^2}{Z^2} & \dfrac{f_y X Y}{Z^2} & \dfrac{f_y X}{Z}
\end{bmatrix}.
$$

부호를 음수로 흡수해 코드에 그대로 박아 넣은 모양이 다음 12개 항이다.

```cpp
// ch13/include/myslam/g2o_types.h L136–L139
_jacobianOplusXi << -fx * Zinv,  0,           fx * X * Zinv2,    fx * X * Y * Zinv2,
                    -fx - fx * X * X * Zinv2,  fx * Y * Zinv,
                     0,           -fy * Zinv,  fy * Y * Zinv2,    fy + fy * Y * Y * Zinv2,
                    -fy * X * Y * Zinv2,        -fy * X * Zinv;
```

`Pose-only` 단항 엣지의 자코비안과 정확히 같은 식이다 — 두 엣지 모두 같은 reprojection 오차를 쓰기 때문에 pose-쪽 chain rule이 동일하다 (참고: `qa-edge-projection-binary.md`).

### 4. `_jacobianOplusXj` (Landmark Jacobian, $2\times 3$)

`_jacobianOplusXi`의 좌측 $2\times 3$ 블록 (= $\partial e / \partial P_c$)을 그대로 재사용해 곱하면 된다.

$$
\frac{\partial e}{\partial \delta P_w} = \frac{\partial e}{\partial P_c} \cdot R_{\text{cam\_ext}} \cdot R_T.
$$

코드는 한 줄.

```cpp
// ch13/include/myslam/g2o_types.h L141–L142
_jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0)
                 * _cam_ext.rotationMatrix() * T.rotationMatrix();
```

`block<2, 3>(0, 0)`이 가리키는 것은 `_jacobianOplusXi`의 **좌측 2×3** = $\partial e / \partial P_c$ (translation 부분). 즉 pose 자코비안의 일부를 landmark 자코비안 계산에 재활용하는 구조다 — 동일한 핀홀 미분을 두 번 계산할 필요가 없다.

> **wasm 빌드 (`ch13-wasm/.../bind_ba.cpp`) 의 등가 코드**
>
> ```cpp
> _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0)
>                  * cam_ext_.linear() * T.linear();
> ```
>
> `Eigen::Isometry3d::linear()`이 회전 블록 (`Sophus::SE3::rotationMatrix()` 와 동일 역할). 둘은 인터페이스만 다르고 의미가 같다 (참고: `qa-sophus-vs-isometry3d.md`).

### 5. 슬롯 인덱스와 정점 순서의 매칭

`_jacobianOplusXi` ↔ `_vertices[0]`, `_jacobianOplusXj` ↔ `_vertices[1]` — 이 매핑은 컴파일러가 강제하지 않으므로 **선언 순서**(`BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ>`)와 **`setVertex(idx, ...)` 호출 순서**가 정확히 일치해야 한다.

```cpp
// backend.cpp 발췌
edge->setVertex(0, vertex_pose);     // → _jacobianOplusXi (2×6)
edge->setVertex(1, vertex_landmark); // → _jacobianOplusXj (2×3)
```

순서를 바꿔 등록하면 BlockSolver가 6×6 자리에 3×3을 끼우려 시도해 SIGSEGV / Eigen assertion 으로 죽거나, 운 나쁘면 묵묵히 잘못된 Hessian이 만들어진다.

### 6. BlockSolver / Schur complement와의 연결

이렇게 채운 두 자코비안이 `BlockSolver_6_3`로 들어가면, BA의 Hessian이 다음 블록 구조로 조립된다.

$$
H = \sum_{e} J_e^\top \Omega_e J_e =
\begin{bmatrix} H_{cc} & H_{cp} \\ H_{cp}^\top & H_{pp} \end{bmatrix},
\quad
H_{cc} \in \mathbb{R}^{6N\times 6N},\;
H_{pp} \in \mathbb{R}^{3M\times 3M},\;
H_{cp} \in \mathbb{R}^{6N\times 3M}.
$$

- $H_{cc}$ 의 각 6×6 대각 블록 ← **`_jacobianOplusXi`** 가 누적
- $H_{pp}$ 의 각 3×3 대각 블록 ← **`_jacobianOplusXj`** 가 누적
- $H_{cp}$ 의 6×3 off-diagonal 블록 ← 두 자코비안의 **교차곱** $J_{Xi}^\top \Omega J_{Xj}$

backend는 $H_{pp}$가 블록 대각이라는 점을 이용해 landmark를 Schur complement로 마진화하고 $6N \times 6N$ 만 sparse Cholesky로 푼다 (참고: `qa-stage1-optimizer-setup.md` §6).

즉 `linearizeOplus`에서 두 슬롯을 정확히 채우는 것이 **BlockSolver_6_3 + Schur 분해**가 작동하기 위한 전제 조건이다.

### 7. Pose-only (단항) 엣지와의 비교

| 엣지 | 베이스 | 채워야 하는 자코비안 |
|------|--------|----------------------|
| `EdgeProjectionPoseOnly` | `BaseUnaryEdge<2, Vec2, VertexPose>`              | **`_jacobianOplusXi` ($2\times 6$)** 한 개 |
| `EdgeProjection`         | `BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ>` | **`_jacobianOplusXi` ($2\times 6$)** + **`_jacobianOplusXj` ($2\times 3$)** |

PnP에서는 landmark가 상수 (`_pos3d`) 이므로 자코비안이 하나뿐, BA에서는 landmark도 변수이므로 두 번째 자코비안이 추가된다. **두 엣지의 `_jacobianOplusXi`는 식이 정확히 같다** — 같은 reprojection 모델의 pose-쪽 미분이기 때문.

### 8. 한눈에 보는 요약

| 슬롯 | 대상 정점 | 차원 | 의미 | 코드 |
|------|-----------|------|------|------|
| `_jacobianOplusXi` | `VertexPose` (`_vertices[0]`) | $2\times 6$ | $\partial e / \partial \delta\xi$, SE(3) twist | 12개 항 직접 대입 |
| `_jacobianOplusXj` | `VertexXYZ` (`_vertices[1]`) | $2\times 3$ | $\partial e / \partial \delta P_w$ | `Xi.block<2,3>(0,0) * R_ext * R_T` |

---

## 한 줄 요약

`EdgeProjection::linearizeOplus`는 — 핀홀 reprojection의 chain rule을 따라 — pose에 대한 **`_jacobianOplusXi` ($2\times 6$)** 와 landmark에 대한 **`_jacobianOplusXj` ($2\times 3$)** 두 개를 채운다. 후자는 전자의 좌측 $2\times 3$ 블록에 $R_{\text{cam\_ext}} R_T$ 만 곱하면 끝.

## 참고

- `ch13/include/myslam/g2o_types.h::EdgeProjection::linearizeOplus` (L123–L143)
- `ch13-wasm/wasm-src/myslam/bindings/bind_ba.cpp::EdgeProjection::linearizeOplus` (L118–L145)
- Gao Xiang, *Visual SLAM 14강* (2판) §7.7.3 식 (7.45)–(7.48) — reprojection 자코비안 유도
- 본 저장소: [`docs/qa-edge-projection-binary.md`](./qa-edge-projection-binary.md), [`docs/qa-pose-only-jacobian.md`](./qa-pose-only-jacobian.md), [`docs/qa-stage1-optimizer-setup.md`](./qa-stage1-optimizer-setup.md), [`docs/qa-sophus-vs-isometry3d.md`](./qa-sophus-vs-isometry3d.md)
