# ch13-wasm — Python 레퍼런스 (13 step 전체)

ch13-wasm 의 visual SLAM 파이프라인을 순수 Python 으로 재구현한 레퍼런스 모음.
WASM/UI 출력과 1:1 비교 가능한 ground-truth 역할 + jupyter 환경 학습 경로.

- **C++/WASM 빌드 불필요** — `opencv-python`, `numpy`, `scipy`, `matplotlib` 만 사용
- **데이터셋 공유** — `../public/datasets/kitti05-mini/` 를 그대로 읽음
- **포맷** — `.py` percent-format (jupytext) → VS Code/Jupyter 에서 "Open as notebook" 가능, 그대로 `python` 실행도 OK
- **범위** — **Step 01 ~ 13 전부**. 각 step 의 verify_*.ts 게이트를 미러

---

## 설치

[uv](https://docs.astral.sh/uv/) 가 필요합니다 (`brew install uv` 또는 [공식 가이드](https://docs.astral.sh/uv/getting-started/installation/) 참조).

```bash
cd ch13-wasm/python
uv sync                # .venv 자동 구성 + 의존성 설치
```

## 실행

### 스크립트 (CI/회귀용)

```bash
uv run python scripts/step01_dataset.py
uv run python scripts/step02_camera.py
uv run python scripts/step03_feature_detection.py
uv run python scripts/step04_stereo_matching.py
uv run python scripts/step05_triangulation.py
uv run python scripts/step06_initial_map.py
uv run python scripts/step07_frame_tracking.py
uv run python scripts/step08_pose_estimation.py
uv run python scripts/step09_keyframe_decision.py
uv run python scripts/step10_new_map_points.py
uv run python scripts/step11_bundle_adjustment.py
uv run python scripts/step12_sliding_window.py
uv run python scripts/step13_full_pipeline.py
```

각 스크립트는 마지막에 `OK — stepNN ...` 를 출력하고 끝납니다. 실패 시 assertion 에서 멈춥니다.

### 노트북 (탐색/시각화용)

```bash
uv run jupyter lab
```

`scripts/stepNN_*.py` 를 우클릭 → **Open With → Jupytext Notebook** 으로 열면 셀 단위 실행 + matplotlib 인라인 시각화가 됩니다.

`# %%` 셀 마커는 [jupytext percent-format](https://jupytext.readthedocs.io/en/latest/formats.html#the-percent-format) — `.py` 파일이지만 git diff 가 깔끔합니다.

### pytest

```bash
uv run pytest -v
```

전 step 의 핵심 assertions 를 한 묶음으로 회귀 검증합니다 (BA·full pipeline 포함하여 약 1 분 내외).

---

## 디렉토리 구조

```
python/
├── pyproject.toml
├── .python-version            # 3.11
├── README.md
├── myslam_ref/                # 공용 알고리즘 모듈
│   ├── dataset.py             # KITTI calib + stereo PNG 로더
│   ├── camera.py              # Pinhole + SE(3) 투영
│   ├── features.py            # GFTT/Harris/FAST/ORB + LK
│   ├── triangulation.py       # LinearSVD + Midpoint
│   ├── pnp.py                 # 4-round outlier 루프 (cv2 LM)
│   ├── keyframe.py            # 3 가지 KF 결정 정책
│   ├── ba.py                  # scipy sparse LM + adaptive chi²
│   ├── se3.py                 # log/exp helpers (mirrors src/lib/slam/se3.ts)
│   ├── slam_map.py            # SlamMap + 4 가지 eviction 정책
│   └── viz.py                 # matplotlib 헬퍼
├── scripts/                   # step01_*.py ~ step13_*.py
└── tests/
    ├── conftest.py
    └── test_smoke.py
```

---

## 13 Step 매핑

| Step | WASM 바인딩 | TS verify | Python ref |
|------|-------------|-----------|------------|
| 01 Dataset             | (없음) | (없음) | `dataset.py`, `scripts/step01_dataset.py` |
| 02 Camera              | `bind_camera.cpp` | (없음) | `camera.py`, `scripts/step02_camera.py` |
| 03 Feature Detection   | `bind_features.cpp` | `verify_features.ts` | `features.py`, `scripts/step03_feature_detection.py` |
| 04 Stereo Matching     | `bind_features.cpp` (LK) | `verify_features.ts` | `scripts/step04_stereo_matching.py` |
| 05 Triangulation       | `bind_triangulation.cpp` | `verify_triangulation.ts` | `triangulation.py`, `scripts/step05_triangulation.py` |
| 06 Initial Map         | features + triangulation | (없음) | `scripts/step06_initial_map.py` |
| 07 Frame Tracking      | `bind_features.cpp` (temporal LK) | (없음) | `scripts/step07_frame_tracking.py` |
| 08 Pose Estimation     | `bind_pnp.cpp` | `verify_pnp.ts` | `pnp.py`, `scripts/step08_pose_estimation.py` |
| 09 Keyframe Decision   | (없음) | (없음) | `keyframe.py`, `scripts/step09_keyframe_decision.py` |
| 10 New Map Points      | features + triangulation | (없음) | `scripts/step10_new_map_points.py` |
| 11 Bundle Adjustment   | `bind_ba.cpp` | `verify_ba.ts` | `ba.py`, `scripts/step11_bundle_adjustment.py` |
| 12 Sliding Window      | (TS only) | `verify_slam.ts` | `se3.py` + `slam_map.py`, `scripts/step12_sliding_window.py` |
| 13 Full Pipeline       | all five modules | (smoke) | `scripts/step13_full_pipeline.py` |

---

## 알고리즘 매핑 메모

- **Step 05 triangulation**: ``Algo.LINEAR_SVD`` 는 DLT (4×4 A 의 SVD → null-space + σ4/σ3 quality). ``Algo.MIDPOINT`` 는 두 ray 의 closest-point.
- **Step 08 PnP**: ``cv2.solvePnP(SOLVEPNP_ITERATIVE)`` 를 inner LM 으로 쓰고, 4 라운드 outlier loop (chi² > 5.991) 와 ``remove_robust_after_round`` 옵션을 Python 으로 감쌌다. g2o + linearizeOplus 와 동일한 noiseless machine precision 수렴을 재현.
- **Step 11 BA**: ``scipy.optimize.least_squares(method="trf", jac_sparsity=...)`` — sparse Jacobian 의 LM 변종. ``adaptive_doublings`` 루프가 inlier_ratio < 0.5 일 때 chi² 임계를 더블링 (verify_ba.ts §D 와 동일).
- **Step 12 SlamMap**: ``src/lib/slam/map.ts`` 의 4 정책 ``ch13-default`` / ``fifo`` / ``covisibility`` / ``distance-only`` 와 ``cleanMap`` 을 그대로 옮겼다. SE(3) log-norm 은 ``se3.py`` 에 좌-Jacobian 역산 폐형식으로 구현.

---

## 새 step 추가 패턴 (참고)

1. `myslam_ref/<topic>.py` 에 알고리즘 본체
2. `scripts/stepNN_<topic>.py` 에 percent-format 셀:
   - `# %% [markdown]` 으로 단계 설명
   - `# %%` 으로 코드 셀
   - 마지막 셀에서 `assert` + `print("OK — stepNN ...")`
3. `tests/test_smoke.py` 에 해당 step 의 핵심 검증을 함수로 등록

---

## 알려진 제약

- ``opencv-python`` 4.13.0 을 사용 — WASM 빌드의 OpenCV 4.13 과 동일 버전 (API 호환).
- ``scipy.optimize.least_squares`` 의 LM/TRF 는 g2o 와 알고리즘적으로 동일 (LM with sparse Jacobian) 이지만 step counting/내부 stopping criteria 가 미세하게 다르다. ``iterations`` 보고 값은 ``nfev`` (residual 평가 횟수) 로 환산.
- KITTI mini 는 5 frame fixture — Step 13 의 정량 trajectory 비교가 매우 짧다. 더 긴 비교가 필요하면 ``public/datasets/kitti-sample/`` (108 frames) 로 ``load_kitti_mini(root=...)`` 호출.
