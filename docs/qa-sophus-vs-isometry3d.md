# Sophus 대신 `Eigen::Isometry3d`를 사용한 단순화

## 문제

Sophus 대신 `Eigen::Isometry3d`를 사용한 단순화는 무엇이며 왜 가능한가?

## 짧은 답

translation에 $V(\phi) \cdot \rho$ 대신 $\rho$를 직접 사용하는 근사를 적용한다. 1차 근사에서 동일하고, LM 수렴 시 step이 작아져 차이가 사라지므로 가능하다.

---

## 상세 설명

### 1. 배경 — SE(3)의 정확한 지수 사상

Sophus의 `SE3::exp(ξ)`는 SE(3)의 수학적으로 엄밀한 지수 사상을 구현한다.

$\xi = (\rho, \phi) \in \mathbb{R}^6$ (앞 3개 $\rho$는 translation 성분, 뒤 3개 $\phi$는 rotation 성분)일 때:

$$
\exp(\xi^\wedge) =
\begin{bmatrix}
\exp(\phi^\wedge) & V(\phi)\,\rho \\
0 & 1
\end{bmatrix}
$$

여기서 핵심은 translation 부분이 **$\rho$가 아니라 $V(\phi)\cdot\rho$** 라는 점이다. $V(\phi)$는 SO(3)의 좌측 자코비안:

$$
V(\phi) = I + \frac{1-\cos\|\phi\|}{\|\phi\|^2}\phi^\wedge + \frac{\|\phi\|-\sin\|\phi\|}{\|\phi\|^3}(\phi^\wedge)^2
$$

즉, se(3) 접공간의 $\rho$와 실제 SE(3)에서의 translation 변위는 $V(\phi)$만큼 회전과 결합된 형태로 변환된다.

### 2. `Eigen::Isometry3d`로 단순화할 때의 차이

`Sophus::SE3` 없이 `Eigen::Isometry3d`만으로 LM 업데이트를 구현하면, 보통 다음과 같이 작성한다:

```cpp
// δξ = [ρ; φ] (6-vector update from solver)
Eigen::Vector3d rho = dx.head<3>();
Eigen::Vector3d phi = dx.tail<3>();

T.linear()      = Sophus::SO3d::exp(phi).matrix() * T.linear();  // 회전은 정확
T.translation() = T.translation() + rho;                          // ★ V(φ)·ρ 대신 ρ
```

회전은 SO(3) 지수로 정확히 합성하지만, **translation은 $V(\phi)$를 빼고 $\rho$만 더하는** 근사를 적용한 것이 핵심이다. 이는 다음과 동치이다:

$$
T_{k+1} \approx
\begin{bmatrix}
\exp(\phi^\wedge) R_k & t_k + \rho \\
0 & 1
\end{bmatrix}
\quad\text{vs Sophus 정확:}\quad
T_{k+1} = \exp(\xi^\wedge) T_k
$$

### 3. 왜 이 근사가 가능한가

#### (1) 1차 근사에서 동일

$\phi \to 0$ 일 때 테일러 전개:

$$
V(\phi) = I + \tfrac{1}{2}\phi^\wedge + \tfrac{1}{6}(\phi^\wedge)^2 + O(\|\phi\|^3)
$$

따라서 $V(\phi)\cdot\rho = \rho + \tfrac{1}{2}\phi \times \rho + O(\|\phi\|^2 \cdot \|\rho\|)$.
**$\|\phi\|$이 작은 영역에서는 $V(\phi)\cdot\rho \approx \rho$가 1차 정확도로 성립**한다. LM/GN이 사용하는 자코비안 자체가 1차 미분 정보이므로, 이 단순화는 자코비안의 정확도와 같은 차수의 근사일 뿐이다.

#### (2) LM 수렴 영역에서 step이 작아짐

Levenberg-Marquardt는 damping $\lambda$를 통해 step 크기를 자체적으로 제어한다. 수렴 근방에서:

- $\|\delta\xi\| \to 0 \;\Rightarrow\; \|\phi\| \to 0$
- 따라서 $\|V(\phi) - I\| = O(\|\phi\|) \to 0$
- 결국 $V(\phi)\cdot\rho$와 $\rho$의 차이는 step 자체보다 더 빠르게 줄어든다 ($O(\|\phi\|\cdot\|\rho\|)$)

즉, 첫 몇 iteration에서는 약간의 경로 차이가 있을 수 있어도, **수렴점은 동일**하다. 최종 잔차 $J(x^*)$는 $\rho$나 $V(\phi)\cdot\rho$ 어느 쪽으로 업데이트했든 같은 fixed point를 만족한다.

#### (3) 자코비안과의 정합성

대부분의 SLAM 코드(특히 g2o 스타일)는 reprojection error의 자코비안을 **R과 t를 분리한 좌측 섭동 모델**로 유도한다:

$$
\frac{\partial e}{\partial \delta\xi} =
\begin{bmatrix}
\frac{\partial e}{\partial \delta\rho} & \frac{\partial e}{\partial \delta\phi}
\end{bmatrix}
$$

이때 $\partial e / \partial \delta\rho$는 "$t$에 $\rho$를 더하면 $e$가 어떻게 변하는가"로 계산되는 경우가 많다 ($V$를 곱하지 않음). 이 자코비안과 `Isometry3d` 단순 업데이트는 **같은 1차 근사 위에서 일관**되므로 LM 수렴이 보장된다. 오히려 자코비안에는 $V$를 무시하고 업데이트에서만 $V$를 적용하면 일관성이 깨져 수렴이 더 나빠질 수 있다.

### 4. 트레이드오프

| 항목 | `Sophus::SE3` | `Eigen::Isometry3d` 단순화 |
|---|---|---|
| 수학적 엄밀성 | SE(3) 지수 사상 정확 | 1차 근사 |
| 큰 step에서의 동작 | 안정적 | 경로가 약간 달라질 수 있음 |
| 수렴점 | 동일 | 동일 (수렴 시) |
| 코드 복잡도 | Sophus 의존 | Eigen만으로 충분 |
| 자코비안과의 일관성 | $V$를 같이 써야 함 | R/t 분리 자코비안과 자연스러움 |

### 5. 결론

**핵심 한 줄**: $V(\phi)\cdot\rho \approx \rho$는 $\|\phi\|$에 대해 1차 오차이고, LM이 수렴 단계에서 $\|\phi\|$을 0으로 보내므로, 자코비안의 1차 근사 정확도와 같은 차수에서 안전하게 무시할 수 있는 항이다. 따라서 Sophus 없이 Eigen만으로도 동일한 최적화 결과를 얻을 수 있다.
