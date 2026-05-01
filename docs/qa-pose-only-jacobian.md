# Pose-only Jacobian $\partial e / \partial \xi$의 차원과 유도 체인

## 문제

pose-only Jacobian $\partial e / \partial \xi$의 차원과 유도 체인은?

## 짧은 답

$2\times 6$ 블록이며, $\partial P_c / \partial \rho = I$, $\partial P_c / \partial \phi = -[P_c]_\times$, $\partial \pi / \partial P_c$의 체인 룰과 $e = z - \hat{z}$의 부호 반전으로 유도된다.

---

## 상세 설명

### 1. 설정과 표기

PnP / Bundle Adjustment의 reprojection error 모델은 다음과 같다.

- $T = (R, t) \in SE(3)$ : 카메라 pose
- $\xi = (\rho, \phi) \in \mathbb{R}^6$ : SE(3)의 Lie algebra 좌표 (앞 3개 $\rho$는 translation 성분, 뒤 3개 $\phi$는 rotation 성분 — 책 컨벤션)
- $P_w \in \mathbb{R}^3$ : world coordinate의 3D 점
- $P_c = R\,P_w + t \in \mathbb{R}^3$ : camera coordinate의 3D 점, 성분 $(X, Y, Z)$
- $\hat{z} = \pi(P_c) \in \mathbb{R}^2$ : 픽셀 투영
- $z \in \mathbb{R}^2$ : 관측 픽셀
- $e = z - \hat{z} \in \mathbb{R}^2$ : reprojection residual

투영 함수는 핀홀 모델:

$$
\pi(P_c) =
\begin{bmatrix}
f_x \dfrac{X}{Z} + c_x \\
f_y \dfrac{Y}{Z} + c_y
\end{bmatrix}
$$

### 2. 차원 결정

| 항목 | 차원 |
|------|------|
| $e$ | $\mathbb{R}^2$ (픽셀 잔차) |
| $\xi$ | $\mathbb{R}^6$ (SE(3) twist) |
| $\partial e / \partial \xi$ | $2 \times 6$ |

따라서 pose-only Jacobian은 항상 $2 \times 6$ 블록이다. landmark-only Jacobian $\partial e / \partial P_w$는 $2 \times 3$이고, full BA에서는 두 블록을 옆에 붙여 $2 \times 9$ 행이 된다.

### 3. 체인 룰 분해

$$
\frac{\partial e}{\partial \xi}
= \frac{\partial e}{\partial P_c} \cdot \frac{\partial P_c}{\partial \xi}
= -\frac{\partial \pi}{\partial P_c} \cdot \frac{\partial P_c}{\partial \xi}
$$

- 부호 $-$ 는 $e = z - \hat{z}$, 즉 $e = z - \pi(P_c)$에서 나온다.
- 차원 검증: $(2 \times 3) \times (3 \times 6) = 2 \times 6$. ✓

이 체인을 세 조각으로 끊어 본다.

### 4. 조각 ① — 투영 야코비 $\partial \pi / \partial P_c$ ($2\times 3$)

$\pi(P_c) = (f_x X/Z + c_x,\ f_y Y/Z + c_y)$를 $X, Y, Z$로 미분:

$$
\frac{\partial \pi}{\partial P_c}
=
\begin{bmatrix}
\dfrac{f_x}{Z} & 0 & -\dfrac{f_x X}{Z^2} \\[6pt]
0 & \dfrac{f_y}{Z} & -\dfrac{f_y Y}{Z^2}
\end{bmatrix}
$$

`Z`가 분모에 있으므로 $Z > 0$ (점이 카메라 앞)인 가정이 필수다.

### 5. 조각 ② — pose 섭동에 대한 $\partial P_c / \partial \xi$ ($3\times 6$)

**Left perturbation 모델**을 사용한다. pose에 작은 섭동 $\delta\xi = (\delta\rho, \delta\phi)$를 좌측에서 곱한다:

$$
T \mapsto \exp(\delta\xi^\wedge)\, T
$$

이때 변환된 점 $P_c'$는

$$
P_c' = \exp(\delta\xi^\wedge)\, T\, P_w
\approx (I + \delta\xi^\wedge)\, P_c
= P_c + \delta\xi^\wedge P_c
$$

여기서 $\delta\xi^\wedge$는 $4\times 4$ 행렬

$$
\delta\xi^\wedge =
\begin{bmatrix}
\delta\phi^\wedge & \delta\rho \\
0 & 0
\end{bmatrix}
$$

이고, $P_c$를 동차좌표 $(P_c, 1)$로 보면

$$
\delta\xi^\wedge\,
\begin{bmatrix} P_c \\ 1 \end{bmatrix}
=
\begin{bmatrix}
\delta\phi^\wedge P_c + \delta\rho \\
0
\end{bmatrix}
=
\begin{bmatrix}
\delta\phi \times P_c + \delta\rho \\
0
\end{bmatrix}
$$

외적 항등식 $a \times b = -b \times a = -[b]_\times a$를 사용하면 $\delta\phi \times P_c = -[P_c]_\times \delta\phi$이므로

$$
P_c' - P_c \;\approx\; \delta\rho \;-\; [P_c]_\times\, \delta\phi
$$

여기서 두 블록을 즉시 읽어낼 수 있다:

$$
\boxed{\;\frac{\partial P_c}{\partial \rho} = I_{3\times 3}, \qquad
\frac{\partial P_c}{\partial \phi} = -[P_c]_\times\;}
$$

따라서 (책의 $\xi = [\rho;\,\phi]$ 순서로)

$$
\frac{\partial P_c}{\partial \xi}
=
\begin{bmatrix} I & -[P_c]_\times \end{bmatrix}
\;\in\; \mathbb{R}^{3 \times 6}
$$

> **주의**: $\xi$의 성분 순서 컨벤션이 책마다 다르다. Gao Xiang 책(SLAM 14강)은 $[\rho;\,\phi]$, 일부 문헌은 $[\phi;\,\rho]$를 쓴다. 순서가 바뀌면 위 두 블록의 좌우 위치도 함께 바뀌므로 코드와 식의 컨벤션을 일치시켜야 한다.

### 6. 조각 ③ — 부호 반전 $e = z - \hat{z}$

$\hat{z} = \pi(P_c)$이므로 $\partial e / \partial P_c = -\partial \pi / \partial P_c$. 이 마이너스가 최종 식 앞에 그대로 남는다.

### 7. 최종 공식

세 조각을 합치면 ($\xi = [\rho;\,\phi]$ 순서):

$$
\frac{\partial e}{\partial \xi}
=
-\frac{\partial \pi}{\partial P_c}
\begin{bmatrix} I & -[P_c]_\times \end{bmatrix}
=
\begin{bmatrix}
-\dfrac{\partial \pi}{\partial P_c} & \dfrac{\partial \pi}{\partial P_c}[P_c]_\times
\end{bmatrix}
$$

전개하면 (SLAM 14강 2판 식 7.46과 동일):

$$
\frac{\partial e}{\partial \xi}
=
-\begin{bmatrix}
\dfrac{f_x}{Z} & 0 & -\dfrac{f_x X}{Z^2} & -\dfrac{f_x XY}{Z^2} & f_x + \dfrac{f_x X^2}{Z^2} & -\dfrac{f_x Y}{Z} \\[6pt]
0 & \dfrac{f_y}{Z} & -\dfrac{f_y Y}{Z^2} & -f_y - \dfrac{f_y Y^2}{Z^2} & \dfrac{f_y XY}{Z^2} & \dfrac{f_y X}{Z}
\end{bmatrix}
$$

좌측 3열이 translation 블록 ($\partial e / \partial \rho$), 우측 3열이 rotation 블록 ($\partial e / \partial \phi$)이다.

### 8. 구현 시 주의

- **섭동 모델 일관성**: 위 유도는 left perturbation 기준이다. 업데이트 식도 $T \leftarrow \exp(\delta\xi^\wedge)\,T$로 통일해야 한다. right perturbation $T \leftarrow T\,\exp(\delta\xi^\wedge)$를 쓰면 $P_c$ 대신 $P_w$ 또는 다른 좌표계 점에 대한 skew-symmetric이 등장하여 식이 달라진다.
- **부호**: 일부 구현은 $e = \hat{z} - z$로 정의한다. 그 경우 위 식에서 전체 부호가 뒤집힌다. 정규방정식 $J^\top J\,\Delta\xi = -J^\top e$의 우변 부호와 짝을 맞춰야 한다.
- **$Z \to 0$ 케이스**: 카메라 앞이 아닌 점이 들어오면 발산한다. outlier rejection / cheirality check가 필요하다.
- **landmark Jacobian과의 관계**: $\partial e / \partial P_w = -\dfrac{\partial \pi}{\partial P_c} R$ ($2\times 3$). pose Jacobian과 합치면 $2 \times 9$가 되어 full BA의 한 행이 된다.

## 참고

- Gao Xiang, *Visual SLAM 14강* (2판) §7.7, 식 7.45–7.46
- Barfoot, *State Estimation for Robotics* §7.1.8 (perturbation models)
- 본 저장소: [`qa-sophus-vs-isometry3d.md`](./qa-sophus-vs-isometry3d.md) — $V(\phi)$ 근사 관련 (translation 업데이트 컨벤션이 위 유도와 어떻게 호환되는지)
