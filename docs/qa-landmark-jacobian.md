# Landmark Jacobian $\partial e / \partial P_w$는 어떻게 계산되는가?

## 문제

`EdgeProjection::linearizeOplus`에서 landmark에 대한 자코비안 `_jacobianOplusXj` ($2\times 3$)는 어떻게 계산되는가?

## 짧은 답

$P_c = R P_w + t$ 이므로 $\partial P_c / \partial P_w = R$ 이다. 따라서 — `_jacobianOplusXi`의 좌측 $2\times 3$ 블록(= $\partial e / \partial P_c$)에 회전 부분만 곱하면 끝.

```cpp
_jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * T.linear();
```

핀홀 미분을 다시 계산할 필요 없이 pose 자코비안의 일부를 그대로 재활용한다.

---

## 상세 설명

### 1. Chain rule 분해

오차는 $e = z - \pi(P_c)$. landmark 자코비안은 chain rule로 두 토막으로 갈린다.

$$
\frac{\partial e}{\partial P_w}
= \underbrace{\frac{\partial e}{\partial P_c}}_{2\times 3,\;\text{핀홀 미분}}
\cdot \underbrace{\frac{\partial P_c}{\partial P_w}}_{3\times 3,\;\text{회전}}.
$$

좌측 $\partial e / \partial P_c$는 **pose 자코비안의 좌측 $2\times 3$ 블록과 정확히 같다** — 두 자코비안이 공유하는 부분이다 (pose 쪽에서는 여기에 $\partial P_c / \partial \delta\xi = [I,\,-P_c^\wedge]$가 곱해진다).

### 2. $\partial P_c / \partial P_w = R$

카메라 좌표가 $P_c = R P_w + t$ 인 강체 변환이라면, $P_w$에 대한 미분은 회전 행렬 $R$ 자체다. 평행이동 $t$는 $P_w$에 대해 상수이므로 떨어진다.

$$
P_c = R P_w + t \;\;\Longrightarrow\;\; \frac{\partial P_c}{\partial P_w} = R \in \mathbb{R}^{3\times 3}.
$$

### 3. 한 줄로 합치면

좌측 $2\times 3$ 블록에 $R$을 곱하기만 하면 된다.

```cpp
_jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * T.linear();
```

- `_jacobianOplusXi.block<2, 3>(0, 0)` — pose 자코비안의 좌측 $2\times 3$ = $\partial e / \partial P_c$ (translation 부분).
- `T.linear()` — `Sophus::SE3` / `Eigen::Isometry3d`에서 $R$ 블록을 꺼내는 표현.

$\partial e / \partial P_c$를 새로 계산하지 않고 `_jacobianOplusXi`의 일부를 재사용하는 것이 핵심이다.

### 4. 본 코드의 변형 — 두 단계 변환

`ch13`의 `EdgeProjection`은 카메라 외부 파라미터 `_cam_ext`까지 끼고 있어 실제 변환이 두 단계다.

$$
P_c = T_{\text{cam\_ext}} \, T \, P_w \;\;\Longrightarrow\;\; \frac{\partial P_c}{\partial P_w} = R_{\text{cam\_ext}} \, R_T.
$$

따라서 코드도 $R$이 두 개 곱해진다.

```cpp
// ch13/include/myslam/g2o_types.h L141–L142
_jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0)
                 * _cam_ext.rotationMatrix() * T.rotationMatrix();
```

```cpp
// ch13-wasm/wasm-src/myslam/bindings/bind_ba.cpp (Eigen::Isometry3d 버전)
_jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0)
                 * cam_ext_.linear() * T.linear();
```

`rotationMatrix()`(Sophus)와 `linear()`(Isometry3d)은 같은 회전 블록을 꺼낸다 (참고: [`qa-sophus-vs-isometry3d.md`](./qa-sophus-vs-isometry3d.md)). `_cam_ext`가 $I$인 단순 모델로 좁히면 짧은 답의 한 줄 식으로 그대로 환원된다.

### 5. 부호 주의

`_error = z − ẑ`이므로 `_jacobianOplusXi`의 12개 항이 통상 공식의 **음수**로 들어가 있다. `_jacobianOplusXj`는 그 음수가 박힌 좌측 $2\times 3$ 블록을 그대로 곱하므로 — 부호 처리는 자동으로 따라온다. 다시 곱하거나 빼지 않는다.

---

## 한 줄 요약

$P_c = R P_w + t$ → $\partial P_c / \partial P_w = R$이므로 핀홀 미분 ($\partial e / \partial P_c$, 이미 `_jacobianOplusXi`의 좌측 $2\times 3$에 들어 있음)에 회전 행렬만 곱하면 된다 — `_jacobianOplusXj = _jacobianOplusXi.block<2,3>(0,0) * T.linear()` 한 줄로 끝.

## 참고

- `ch13/include/myslam/g2o_types.h::EdgeProjection::linearizeOplus` (L123–L143)
- `ch13-wasm/wasm-src/myslam/bindings/bind_ba.cpp::EdgeProjection::linearizeOplus`
- 본 저장소: [`docs/qa-edge-projection-jacobians.md`](./qa-edge-projection-jacobians.md), [`docs/qa-pose-only-jacobian.md`](./qa-pose-only-jacobian.md), [`docs/qa-sophus-vs-isometry3d.md`](./qa-sophus-vs-isometry3d.md)
