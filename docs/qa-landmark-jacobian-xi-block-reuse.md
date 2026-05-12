# landmark Jacobian에서 `_jacobianOplusXi`의 앞 3열을 쓸 수 있는 이유는?

## 문제

`EdgeProjection::linearizeOplus`에서 landmark 자코비안 `_jacobianOplusXj`를 계산할 때, pose 자코비안 `_jacobianOplusXi`의 **좌측 $2\times 3$ 블록**(= 앞 3열)을 그대로 가져다 쓰는 코드가 등장한다.

```cpp
_jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0)
                 * _cam_ext.rotationMatrix() * T.rotationMatrix();
```

새로 핀홀 미분을 계산하지 않고 — 이미 채워둔 `_jacobianOplusXi`의 일부를 재활용하는 — 이 트릭이 **수학적으로 왜 성립**하는가?

## 짧은 답

g2o의 SE(3) 좌측 perturbation 컨벤션 $\delta\xi = (\rho, \phi)$에서 **앞 3열 = translation 블록**이고, $\partial P_c / \partial \rho = I$ 이므로 chain rule이 다음과 같이 정확히 떨어지기 때문이다.

$$
\underbrace{\frac{\partial e}{\partial \rho}}_{\text{Xi의 앞 3열}}
\;=\;
\underbrace{\frac{\partial e}{\partial P_c}}_{2\times 3,\;\text{핀홀 미분}}
\cdot
\underbrace{\frac{\partial P_c}{\partial \rho}}_{=\,I_3}
\;=\;
\frac{\partial e}{\partial P_c}.
$$

즉 — pose 자코비안의 앞 3열이 **그 자체로** $\partial e / \partial P_c$ 이다. landmark 자코비안 chain rule
$\partial e / \partial P_w = (\partial e / \partial P_c) \cdot R$
에서 좌측 항으로 그대로 재사용할 수 있다.

---

## 상세 설명

### 1. g2o SE(3) twist의 컨벤션 — `(ρ, φ)` 순서

`Sophus::SE3` / g2o `VertexSE3Expmap`은 6-벡터 perturbation을 다음 순서로 본다.

$$
\delta\xi = \begin{pmatrix} \rho \\ \phi \end{pmatrix} \in \mathbb{R}^6,
\qquad
\rho \in \mathbb{R}^3 \;(\text{translation}), \quad
\phi \in \mathbb{R}^3 \;(\text{rotation, so(3)}).
$$

그래서 `_jacobianOplusXi` ($2\times 6$)의 **열 인덱스 0–2가 translation**, **3–5가 rotation**이다.

```
        ┌─────────────────────────────────────┐
        │  ρ (col 0..2)   │   φ (col 3..5)    │   ← _jacobianOplusXi (2×6)
        │   ∂e/∂ρ         │   ∂e/∂φ           │
        └─────────────────────────────────────┘
                ▲
                └── _jacobianOplusXi.block<2,3>(0,0) 가 가리키는 부분
```

### 2. SE(3) 좌측 perturbation의 핵심 항등식

$P_c = T P_w$ 에 좌측 perturbation $T \leftarrow \exp(\delta\xi^\wedge) T$ 를 적용해 1차 항까지 전개하면 (Visual SLAM 14강 §4.3, 식 4.27 변형),

$$
\frac{\partial P_c}{\partial \delta\xi}
=
\begin{bmatrix}
I_3 & -P_c^{\wedge}
\end{bmatrix}
\in \mathbb{R}^{3\times 6}.
$$

블록을 컬럼별로 쪼개면 의미가 또렷해진다.

| 블록 | 컬럼 인덱스 | 값 | 의미 |
|------|-------------|----|------|
| translation 부분 | 0–2 | $\partial P_c / \partial \rho = I_3$ | translation perturbation $\rho$를 더하면 $P_c$도 그만큼 그대로 이동 |
| rotation 부분 | 3–5 | $\partial P_c / \partial \phi = -P_c^\wedge$ | so(3) perturbation $\phi$는 $P_c$를 $-P_c^\wedge$ 방향으로 회전시킴 |

**translation 부분이 단위행렬**이라는 점이 본 질문의 핵심 — 이것이 "앞 3열 = $\partial e / \partial P_c$"라는 등식을 만든다.

### 3. Chain rule 한 번 더 — 앞 3열만 떼어 보기

pose 자코비안 전체는

$$
\frac{\partial e}{\partial \delta\xi}
= \frac{\partial e}{\partial P_c} \cdot \frac{\partial P_c}{\partial \delta\xi}
= \frac{\partial e}{\partial P_c} \cdot \begin{bmatrix} I_3 & -P_c^\wedge \end{bmatrix}
= \begin{bmatrix} \dfrac{\partial e}{\partial P_c} \cdot I_3 \;\big|\; \dfrac{\partial e}{\partial P_c} \cdot (-P_c^\wedge) \end{bmatrix}.
$$

좌측 $2\times 3$ 블록만 떼면

$$
\boxed{\;\frac{\partial e}{\partial \rho} = \frac{\partial e}{\partial P_c} \cdot I_3 = \frac{\partial e}{\partial P_c}\;}
$$

— **곱셈이 단위행렬에 막혀 그대로 통과**한다. 따라서 `_jacobianOplusXi`의 앞 3열에 들어 있는 수치가 곧 핀홀 투영 미분 $\partial e / \partial P_c$ 그 자체이다.

코드의 12개 항 중 좌측 6개 (translation 컬럼) 와 표준 핀홀 미분이 그대로 일치한다.

```cpp
// _jacobianOplusXi의 좌측 2×3 (col 0..2) — = -∂π/∂P_c
//   ┌                                          ┐
//   │ -fx/Z      0       fx*X/Z²               │
//   │  0       -fy/Z     fy*Y/Z²               │
//   └                                          ┘
// 이것이 곧 ∂e/∂P_c (e = z - π(P_c) 부호 포함).
```

`-` 부호가 모두 박혀 있는 이유는 `_error = z − ẑ` 이라서 — pose Jacobian, landmark Jacobian, 두 자리 모두 같은 음의 부호를 그대로 물려받는다 (재계산 / 부호 뒤집기 불필요).

### 4. landmark 자코비안에서 어떻게 쓰이나

landmark 자코비안의 chain rule은

$$
\frac{\partial e}{\partial P_w}
= \frac{\partial e}{\partial P_c} \cdot \frac{\partial P_c}{\partial P_w}
= \frac{\partial e}{\partial P_c} \cdot R.
$$

좌측 $\partial e / \partial P_c$가 필요하다. 그런데 §3에서 보았듯이 **이 행렬이 이미 `_jacobianOplusXi`의 앞 3열에 그대로 들어 있다.** 그래서

```cpp
_jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0)   // = ∂e/∂P_c (앞 3열)
                 * _cam_ext.rotationMatrix() * T.rotationMatrix(); // = R
```

핀홀 미분의 6개 항 (fx, fy, X, Y, Z 조합)을 **두 번 계산하지 않는다** — 캐시 히트.

### 5. 만약 컨벤션이 `(φ, ρ)` 였다면?

g2o가 아니라 일부 라이브러리처럼 rotation 먼저, translation 나중인 컨벤션을 따랐다면

$$
\frac{\partial P_c}{\partial \delta\xi}
=
\begin{bmatrix} -P_c^{\wedge} & I_3 \end{bmatrix}
\;\Longrightarrow\;
\text{앞 3열} = \frac{\partial e}{\partial P_c} \cdot (-P_c^\wedge) \neq \frac{\partial e}{\partial P_c}.
$$

이 경우엔 **뒤 3열**(`block<2,3>(0,3)`)이 $\partial e / \partial P_c$가 된다. 즉 본 코드의 `block<2,3>(0,0)`은 **g2o `(ρ, φ)` 컨벤션에 강하게 의존**한다 — 컨벤션이 바뀌면 블록 오프셋도 따라 바뀌어야 한다.

> Sophus 0.x → 1.x 마이그레이션이나 다른 SE(3) 라이브러리로 옮길 때 주의할 포인트. 자코비안이 통째로 어긋난 것처럼 보이는데 알고 보면 "앞 3열 / 뒤 3열" 순서만 뒤바뀐 경우가 흔하다.

### 6. 한눈에 보는 매핑

| 식 | 차원 | `_jacobianOplusXi`에서 어디 | 비고 |
|----|------|------------------------------|------|
| $\partial P_c / \partial \rho$ | $3\times 3$ | (개념상 곱해진 후 사라짐) | $= I_3$ 이라 **곱셈이 항등** |
| $\partial e / \partial P_c$ | $2\times 3$ | **`block<2,3>(0,0)`** | 핀홀 미분, 캐시 대상 |
| $\partial P_c / \partial \phi$ | $3\times 3$ | (개념상) | $= -P_c^\wedge$ |
| $\partial e / \partial \phi$ | $2\times 3$ | `block<2,3>(0,3)` | landmark 계산에는 안 씀 |
| $\partial P_c / \partial P_w$ | $3\times 3$ | (별도) | $= R_{\text{cam\_ext}} R_T$ |

landmark 자코비안은 위 표에서 **2번째 행 × 5번째 행** — 두 토막을 곱해 만든다.

---

## 한 줄 요약

g2o `(ρ, φ)` SE(3) 컨벤션에서 $\partial P_c / \partial \rho = I_3$ 이므로 chain rule이 단위행렬에 막혀 통과해 `_jacobianOplusXi`의 **앞 3열 그 자체가 $\partial e / \partial P_c$** 가 된다 — landmark 자코비안 $\partial e / \partial P_w = (\partial e / \partial P_c) \cdot R$ 이 그 블록을 그대로 재사용할 수 있는 근거.

## 참고

- `ch13/include/myslam/g2o_types.h::EdgeProjection::linearizeOplus` (L123–L143)
- `ch13-wasm/wasm-src/myslam/bindings/bind_ba.cpp::EdgeProjection::linearizeOplus` (L118–L145)
- Gao Xiang, *Visual SLAM 14강* (2판) §4.3 식 4.26–4.27 (좌측 perturbation의 $[I,\,-P_c^\wedge]$), §7.7.3 식 (7.45)–(7.48) (reprojection 자코비안)
- 본 저장소: [`docs/qa-landmark-jacobian.md`](./qa-landmark-jacobian.md), [`docs/qa-edge-projection-jacobians.md`](./qa-edge-projection-jacobians.md), [`docs/qa-pose-only-jacobian.md`](./qa-pose-only-jacobian.md)
