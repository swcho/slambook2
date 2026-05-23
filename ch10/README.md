역할

g2o의 BaseBinaryEdge::computeError()를 오버라이드하여, 두 SE(3) 포즈 정점(v1, v2) 사이의 관측된 상대 변환(_measurement)이 실제 추정값과 얼마나 어긋나는지를 6차원 리 대수 벡터로 반환한다. 이 잔차(residual)가 작아지도록 Levenberg-Marquardt가 모든 정점을 최적화한다.

수식 풀이

각 항의 의미:
- v1, v2 : 두 정점의 현재 추정 포즈 $T_1, T_2 \in SE(3)$ (월드→로컬)
- _measurement : 엣지에 저장된 관측 상대 변환 $\hat{T}_{12}$ (예: 오도메트리, 루프 클로저로 얻은 1→2 변환)
- v1.inverse() * v2 : 현재 추정으로 계산한 상대 변환 $T_1^{-1} T_2$
- _measurement.inverse() * (v1.inverse() * v2) : 관측을 기준으로 본 "잔차 변환"
$$\Delta T = \hat{T}_{12}^{-1} , T_1^{-1} T_2$$
- .log() : $SE(3) \to \mathfrak{se}(3)$ 매핑으로 6차원 벡터 $\xi \in \mathbb{R}^6$ 추출

따라서 잔차는

$$e_{ij} = \ln!\bigl(\hat{T}_{ij}^{-1} , T_i^{-1} T_j\bigr)^{\vee}$$

이상적으로 $T_1^{-1} T_2 = \hat{T}_{12}$이면 $\Delta T = I$ 이고 $\log(I) = 0$ 이 된다. 즉 상대 포즈 추정이 관측과 정확히 일치할 때 오차 0이 되는 자연스러운 정의이다.

왜 리 대수로 쓰는가

- g2o는 잔차를 벡터(여기선 Vector6d)로 요구한다. SE(3)는 매니폴드이므로 단순한 행렬 뺄셈은 의미가 없고, log()로 접공간에 사상해야 미분·정보 행렬 곱 등이 합당해진다.
- 이렇게 정의한 오차로 인해 같은 파일 linearizeOplus() (124–131행)에서 야코비안이 $-J, \mathrm{Adj}(T_2^{-1})$, $J,\mathrm{Adj}(T_2^{-1})$ 형태로 깔끔하게 유도된다 (책 10장 유도와 일치).

관련 위치

- 좌곱 업데이트 규칙: oplusImpl (ch10/pose_graph_g2o_lie_algebra.cpp:70) — _estimate = exp(δξ) * _estimate
- 위 오차와 짝을 이루는 야코비: linearizeOplus (ch10/pose_graph_g2o_lie_algebra.cpp:124)
- $J_r^{-1}$ 근사: JRInv (ch10/pose_graph_g2o_lie_algebra.cpp:29) — 실제로는 36행에서 단위행렬로 근사하여 사용