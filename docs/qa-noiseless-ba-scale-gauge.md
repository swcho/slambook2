# Noiseless BA에서 chi²가 10⁻²²인데 maxTr이 10⁻³인 이유는?

## 문제

Synthetic noiseless 데이터로 ch13 Stage 2 backend BA를 돌렸더니 다음과 같은 결과가 나온다.

```
chi²   = 1.2e-22     ← 사실상 numerical zero
maxTr  = 8.7e-04     ← translation residual은 mm 단위로 살아있음
maxRot = 3.1e-09     ← rotation은 거의 0
```

residual이 numerical zero까지 떨어졌는데도 **추정된 translation이 ground truth에서 0.001 단위로 벗어나 있다**. 왜 그런가?

## 짧은 답

**모노큘러 BA의 scale gauge 자유도** 때문이다.
모노큘러 reprojection BA의 cost function은 **sim(3) 변환(=SE(3) + scale)** 하에 불변(invariant)이다 → **gauge DOF = 7개** (3 translation + 3 rotation + 1 scale).
1개의 pose만 fixed로 두면 그 pose의 SE(3) 6-DOF는 잠기지만 **scale 1-DOF는 여전히 자유**. 따라서 다음과 같은 동치류 안의 어떤 해라도 chi²를 똑같이 0으로 만든다.

$$
T_i' = (s\,R_w T_i,\ s\,R_w \mathbf{t}_i),\qquad \mathbf{p}_j' = s\,R_w \mathbf{p}_j
$$

→ rotation residual은 scale에 영향받지 않으므로 0에 가깝지만, translation은 `s ≠ 1` 인 만큼 ground truth에서 비례적으로 어긋난다. **이것이 정확히 maxTr이 살아있는 양상이다.**

> Stereo/RGB-D BA에서는 baseline이 절대 scale을 박아주므로 이 현상이 나타나지 않는다.

---

## 상세 설명

### 1. Gauge freedom이란

비선형 최소제곱 BA에서

$$
\chi^2(\theta) = \sum_{(i,j)} \big\| \pi(K, T_i, \mathbf{p}_j) - \mathbf{u}_{ij} \big\|^2
$$

cost function이 **연속 변환군 G의 작용에 대해 불변**일 때, G가 그 BA 문제의 **gauge group**이다. 즉 임의의 $g \in G$ 에 대해

$$
\chi^2(\theta) = \chi^2(g \cdot \theta)
$$

이 성립하면 $\theta^*$ 와 $g \cdot \theta^*$ 는 모두 똑같이 cost를 최소화하므로 **해가 한 점이 아니라 G만큼의 manifold(다양체)** 가 된다.

| BA 종류 | Gauge group | Gauge DOF |
|---------|------------|-----------|
| 모노큘러 BA | sim(3) = SE(3) × ℝ⁺ | **7** |
| Stereo / RGB-D BA | SE(3) | **6** |
| Pose-graph (loop) | SE(3) | 6 |

모노큘러에서 scale이 추가되는 이유: 모든 pose의 translation과 모든 landmark 좌표를 동일한 양의 상수 s로 곱해도 reprojection은 똑같이 나온다. 카메라가 “2배 크고 2배 멀리” 있어도 픽셀 위에 같은 점이 찍힌다.

### 2. Hessian 관점: gauge DOF는 H의 null space

정규방정식 $H\,\Delta x = -b$ 에서 gauge freedom은 **$H$의 정확한 rank-deficiency**로 나타난다.

- 모노큘러 noiseless BA: $\mathrm{null}(H) = 7$
- Stereo noiseless BA:   $\mathrm{null}(H) = 6$
- Gauge가 모두 잠긴 BA:  $\mathrm{null}(H) = 0$

null space의 vector는 cost에 영향을 안 주는 방향이므로 LM 단계에서 그 방향으로 얼마든지 이동할 수 있고, 댐핑 $\lambda$ 만이 그 방향을 “0 근처로” 잡아둘 뿐이다.

> 실제로는 floating-point 잡음 + LM 댐핑 때문에 null이 정확히 7-rank-deficient는 아니지만 "**near-null space에서 conditioning이 매우 나쁜 7개 방향**"으로 드러난다 → 그 방향의 residual은 chi²에 거의 기여하지 않으므로 chi²는 0으로 떨어지지만 그 방향의 parameter error는 임의로 클 수 있다.

### 3. Pose 1개를 fix하면 무엇이 잠기는가

ch13 backend는 활성 윈도우 안의 가장 오래된 keyframe 1개를 `setFixed(true)` 로 두는 것이 일반적이다. 이 한 개 pose가 잠기면 SE(3) 6-DOF (rotation + translation)가 사라진다.

| 잠긴 DOF | 남는 gauge DOF (모노) | 남는 gauge DOF (스테레오) |
|---------|----------------------|---------------------------|
| 0개     | 7 (SE(3) + s)       | 6 (SE(3)) |
| **1개 pose fixed** | **1 (s만)** | **0 (모두 잠김)** |
| 2개 pose fixed | 0 | 0 (over-constrained) |

→ **모노큘러는 1개 pose fix만으로는 부족**. scale 1-DOF가 남아 translation residual이 살아있다.

### 4. 왜 chi²와 maxTr이 따로 노는가

reprojection $\pi(T_i, \mathbf{p}_j) = K\,\frac{R_i \mathbf{p}_j + \mathbf{t}_i}{[R_i \mathbf{p}_j + \mathbf{t}_i]_z}$ 는 scale에 대해 **0차 동차(scale-invariant)**:

$$
\pi(s R_i, s \mathbf{t}_i, s \mathbf{p}_j) = K\,\frac{s(R_i \mathbf{p}_j + \mathbf{t}_i)}{s[R_i \mathbf{p}_j + \mathbf{t}_i]_z}
= \pi(R_i, \mathbf{t}_i, \mathbf{p}_j)
$$

→ scale은 reprojection 잔차에 들어오지 않는다 → **chi²에 잡히지 않는다**.

반면 ground truth와의 비교는 절대 좌표계에서 이뤄지므로

$$
\Delta \mathbf{t}_i = \mathbf{t}_i^{\text{est}} - \mathbf{t}_i^{\text{gt}} = (s - 1)\,\mathbf{t}_i^{\text{gt}} + \cdots
$$

→ scale이 1에서 $\varepsilon$ 만큼 어긋나면 **translation 오차는 $\|\mathbf{t}\|$ 의 $\varepsilon$ 배**. 카메라 baseline이 m 단위이고 $\varepsilon \sim 10^{-3}$ 이면 정확히 maxTr ≈ 10⁻³ 가 나온다.

Rotation은 어떤가? scale 작용은 rotation 부분 $R_i$ 에 영향을 안 준다 (`s R = R\cdot s`처럼 회전과 스칼라는 가환이라 $R$은 그대로). → **maxRot은 chi²와 같이 numerical zero**까지 떨어진다. 위 결과의 maxRot ≈ 3e-09 가 정확히 그 모습.

이것이 **chi² ≈ 0, maxRot ≈ 0, maxTr ≠ 0** 의 비대칭이 갖는 의미다. 이 패턴을 보는 순간 “scale gauge가 안 잡혔구나”라고 진단할 수 있다.

### 5. 진단: 정말 scale gauge 때문인지 확인하는 법

**(a) sim(3) 정렬 후 비교**
추정 결과 $\{T_i^{\text{est}}, \mathbf{p}_j^{\text{est}}\}$ 와 ground truth를 sim(3) 변환(7-DOF)으로 정렬한 뒤 잔차를 다시 계산. 정렬 후 maxTr이 **chi²와 같은 수준 (10⁻¹⁰ 이하)** 으로 떨어지면 gauge 문제가 맞다.

```python
# 의사코드
s, R, t = umeyama(t_est, t_gt)         # sim(3) 정렬
t_aligned = s * R @ t_est + t
maxTr_aligned = max(norm(t_aligned - t_gt))  # 이게 ~0이면 gauge 문제 확정
```

**(b) scale 비율 확인**
$s = \|\mathbf{t}^{\text{est}}\| / \|\mathbf{t}^{\text{gt}}\|$ 가 1에 가깝지만 정확히 1은 아니다 (예: 1.0008). 이 한 숫자가 maxTr의 출처.

**(c) Hessian의 최소 고유값**
LM 종료 시점에서 $H$ 의 가장 작은 7개 고유값이 다른 고유값에 비해 **압도적으로 작으면** (보통 `1e-10` 이하 vs. `1e+3` 이상) gauge null space의 흔적.

### 6. 처방

| 방법 | 잠그는 것 | 모노큘러에 충분? |
|------|----------|------------------|
| Pose 1개 fix | SE(3) 6-DOF | ❌ scale 1개 남음 |
| **Pose 2개 fix** | SE(3) + 두 카메라 사이 거리 = scale | ✅ |
| Pose 1개 fix + landmark 1개 fix | SE(3) + scale | ✅ |
| Pose 1개 fix + 두 pose 사이 거리 prior edge | SE(3) + scale | ✅ |
| Stereo/RGB-D edge 사용 | baseline이 scale 박음 | ✅ (자동) |
| sim(3) gauge prior (free-gauge 방식) | 7-DOF 정규화 | ✅ |

ch13 코드는 stereo edge를 쓰기 때문에 **실 운영에서는 이 문제가 발생하지 않는다**. 하지만 **synthetic noiseless 모노큘러 unit test**를 짤 때는 위 처방 중 하나를 반드시 적용해야 chi²와 함께 maxTr도 떨어지는 것을 확인할 수 있다.

### 7. 흔한 오해

- ❌ **“chi²가 떨어졌으니 BA가 옳게 풀렸다.”** → 모노큘러에서는 **틀린 결론**. chi²는 gauge에 무감각하므로 7개 방향의 오차를 못 본다.
- ❌ **“pose 하나 fixed면 충분하지 않나?”** → SE(3) 문제에선 OK, 모노큘러 BA에선 NO.
- ❌ **“landmark가 많으니 scale은 자동으로 잡히겠지.”** → 잡히지 않는다. landmark도 같이 sim(3)으로 변환되면 reprojection이 **bit-exact 동일**.
- ❌ **“rotation도 같이 어긋나야 자연스럽지 않나?”** → 아니다. scale은 회전과 가환이라 회전만 따로 정확히 풀린다. **maxRot ≈ 0 + maxTr ≠ 0** 의 “비대칭 잔차”가 sim(3) gauge의 사인쳐(signature).

---

## 요약

- 모노큘러 BA cost는 **sim(3)** 에 불변 → gauge DOF = **7**.
- pose 1개 fix는 SE(3) 6개만 잠금 → **scale 1-DOF가 자유**.
- Reprojection이 scale-invariant이므로 **chi²는 scale 오차를 못 본다** → 0까지 떨어진다.
- 절대 좌표계에서의 translation 오차 $\Delta \mathbf{t} \approx (s-1)\,\mathbf{t}$ 만 살아남아 **maxTr ≈ 10⁻³** 형태로 보인다.
- Rotation은 scale과 가환이므로 **maxRot은 chi²와 같이 0으로 떨어진다** → “maxTr만 큼” 비대칭이 진단 단서.
- 처방: 두 번째 pose fix, landmark fix, distance prior, stereo edge, sim(3) gauge prior 중 하나 도입.
