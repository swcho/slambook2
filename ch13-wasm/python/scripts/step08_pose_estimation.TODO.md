# Step 08 — `estimate_pose` 4-way 비교 구현

## 목적
`bind_pnp.cpp` 의 **g2o + RobustKernelHuber + 4-round outlier loop** 를 Python 에서 어디까지 충실히 재현할 수 있는지, 4 가지 접근을 **각각 별도 함수**로 구현해 동일 시나리오로 비교한다.

| 변종 | 위치 | inner LM | Huber 방식 |
|---|---|---|---|
| 1. cv2 (baseline) | `myslam_ref/pnp.py::estimate_pose_cv2` | `cv2.solvePnP(SOLVEPNP_ITERATIVE)` | mask freeze 근사 (현행) |
| 2. scipy | `myslam_ref/pnp.py::estimate_pose_scipy` | `scipy.optimize.least_squares(loss='huber')` | scipy 내장 (진짜 Huber) |
| 3. handwritten | `myslam_ref/pnp_lm.py::estimate_pose_handwritten` | 직접 작성한 SE(3) LM | IRLS weight (g2o 와 동일) |
| 4. g2o | `myslam_ref/pnp.py::estimate_pose_g2o` | `g2o-python` 또는 `gtsam` | 라이브러리 내장 |

공통 시그니처 / 반환 = 기존 `estimate_pose` (`PnPResult`) 와 동일.

---

## 0. 공통 유틸 (`myslam_ref/pnp.py`)
- [ ] `_se3_left_update(T, xi)` — `bind_pnp.cpp::oplusImpl` 미러. translation = head(3), rotation = tail(3), `dT * T`.
- [ ] `_project(T, pts3, K)` → `(N, 2)` uv + `(N,)` per-edge chi².
- [ ] `_pose_jacobian(T, Pw, K)` → `(2, 6)` — `linearizeOplus` 의 해석 야코비안 (`bind_pnp.cpp:85-106` 그대로).
- [ ] `_huber_weight(r2, delta2)` — g2o Huber: `|r| ≤ δ → 1`, 그 외 `δ/|r|`.
- [ ] `_run_4round_loop(inner_solve_fn, ...)` — chi² > threshold 마스킹 + `remove_robust_after_round` 토글. inner solver 만 갈아 끼우면 변종이 공유.
- [ ] 기존 `estimate_pose` 를 `estimate_pose_cv2` 로 rename 하고 module-level alias `estimate_pose = estimate_pose_cv2` 유지 (step01~13 스크립트 호환).

---

## 1. `estimate_pose_cv2` (baseline)
- [ ] 현재 `estimate_pose` 본문 그대로, 이름만 변경.
- [ ] docstring 에 "Huber 가 진짜가 아닌 mask-freeze 근사" 임을 명시.

## 2. `estimate_pose_scipy`
- [ ] inner = `scipy.optimize.least_squares(residual, x0=pose6, jac=jac, loss='huber', f_scale=√chi2_threshold, method='trf', max_nfev=iter_per_round*6)`.
- [ ] `residual(pose6)` — 현재 inlier 만 골라 `(2K,)` 평탄화.
- [ ] `jac(pose6)` — `_pose_jacobian` 인라이어 별로 조립 → `(2K, 6)` dense (K 가 크지 않으니 sparse 불필요).
- [ ] kernel drop 이후 라운드는 `loss='linear'` 로 전환.
- [ ] iteration 수는 `result.nfev` 를 `iter_per_round` 단위로 환산해 `round_iters` 에 기록.

## 3. `estimate_pose_handwritten` — 별도 모듈 `myslam_ref/pnp_lm.py`
- [ ] LM 메인 루프 (1 라운드 = `iter_per_round` 회 iteration):
  1. residual `r ∈ ℝ^{2N}` + Jacobian `J ∈ ℝ^{2N×6}` 조립
  2. kernel 활성 라운드면 `W = diag(_huber_weight(r²ᵢ, δ²))` 적용 (IRLS)
  3. normal eq: `(JᵀWJ + λI) δ = JᵀW r` → `np.linalg.solve`
  4. trial pose `T' = _se3_left_update(T, δ)`; cost 감소 시 채택 + `λ /= 10`, 아니면 reject + `λ *= 10`
- [ ] 초기 `λ = 1e-3 · max(diag(JᵀJ))` (g2o LM 초기화와 동일 휴리스틱).
- [ ] 라운드 전이는 `_run_4round_loop` 가 처리 — 이 함수는 내부 루프만 책임.

## 4. `estimate_pose_g2o`
- [ ] 의존성 결정 (`uv add` 로 직접 시도):
  - **Option a: `g2o-python`** — `bind_pnp.cpp` 와 가장 직설적인 매핑. macOS arm64 wheel 가용 여부 확인.
  - **Option b: `gtsam`** — pip wheel 안정적, custom factor 로 동일 그래프 표현.
  - [ ] 둘 다 실패하면 이 변종은 skip 하고 비교표에 `N/A` 로 표기.
- [ ] `VertexPoseSE3` + `EdgeReprojectionPoseOnly` 를 라이브러리 vertex/factor 로 1:1 옮김 (BlockSolver 6×3, LinearSolverDense, OptimizationAlgorithmLevenberg, RobustKernelHuber).
- [ ] `setLevel(1)` outlier 토글 + `removeKernelAfterRound` 그대로 미러 (`bind_pnp.cpp:268-310`).

---

## 5. 비교 testbench (`scripts/step08_pose_estimation.py`)
- [ ] 기존 A/B/C/D 시나리오 루프를 `run_gate(estimate_fn) -> dict` 로 추출.
- [ ] 4 함수에 동일 seed 셋으로 돌려 결과 표 출력:
  ```
  variant       | A rotErr | B rotErr | C rotErr | C recall | D pass | wallclock
  ----------------------------------------------------------------------------------
  cv2           | ...      | ...      | ...      | ...      | ...    | ...
  scipy         | ...      | ...      | ...      | ...      | ...    | ...
  handwritten   | ...      | ...      | ...      | ...      | ...    | ...
  g2o           | ...      | ...      | ...      | ...      | ...    | ...
  ```
- [ ] matplotlib 4-panel: 라운드별 inlier_count bar 비교.
- [ ] matplotlib 4-panel: 최종 per-edge chi² 히스토그램 (`viz.draw_residual_histogram` 재사용).
- [ ] 끝에 cross-check: 4 변종 모두 §A 에서 rotErr < 1e-6, T_cw pairwise diff < 1e-3 이면 PASS.

## 6. 회귀 테스트 (`tests/test_smoke.py`)
- [ ] §A noiseless 에서 4 변종 모두 machine precision 으로 수렴하는지 assert.
- [ ] §C outlier recall 이 모두 ≥ 80 % 인지 assert (g2o 변종은 skip 허용).

## 7. README (`ch13-wasm/python/README.md`)
- [ ] L113 "Step 08 PnP" 문단을 4-way 비교 결과로 갱신.
- [ ] 진짜 Huber vs mask-freeze 근사의 차이를 학습자에게 명시.
- [ ] g2o 의존성 설치 노트 (어느 패키지를 썼는지, macOS / linux 별 가용성).

---

## 의사결정 보류
- **g2o 백엔드**: `g2o-python` 우선, 실패 시 `gtsam`, 둘 다 실패 시 N/A. 실제 `uv add` 결과 확인 후 확정.
- **handwritten 위치**: `pnp.py` 안에 두면 한 파일이 너무 커짐. 별도 `pnp_lm.py` 권장 (LM 학습 자료로도 활용).
- **stress test 추가 여부**: 4 변종이 §A~D 에서 모두 동일 결과면 분별력이 떨어짐. 그 경우 5 px noise / 40 % outlier 시나리오를 비교 전용으로 추가.

## 작업 순서 권장
1. §0 공통 유틸 → 2. §1 cv2 rename → 3. §3 handwritten (가장 학습 가치 큼) → 4. §2 scipy → 5. §4 g2o (의존성 리스크 큼, 마지막) → 6. §5 비교 testbench → 7. §6 테스트 / §7 README.
