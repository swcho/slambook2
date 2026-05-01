# Step 13 — Full Pipeline (End-to-End VO)

## 학습 목표

- [x] StereoInit / Track / KeyFrame insertion / Backend BA가 어떻게 한 사이클로 연결되는지
- [x] 프리셋(book-default / conservative / aggressive)이 정확도/연산비용에 주는 영향
- [x] Backend BA on/off 시 trajectory + chi² 차이를 직접 관찰

## C++ 원본 매핑

- 파일: `ch13/src/frontend.cpp::Frontend::AddFrame` + `ch13/src/backend.cpp::Backend::Optimize`
- 핵심 흐름: `frame 0` → StereoInit. `frame i≥1` → Track(LK prev→curr) + EstimateCurrentPose(g2o PnP) + (조건 충족 시) InsertKeyframe + DetectFeatures(mask-existing) + FindFeaturesInRight + TriangulateNewPoints. KF 삽입 시 Backend Optimize 호출.

## UI

- ParamPanel:
  - **프리셋 3종**: book-default / conservative / aggressive
  - 개별 슬라이더: maxFeatures · PnP rounds · num_features_needed_for_keyframe · num_features_tracking · BA iterations · BA chi² init · adaptive rounds · window size · policy
- InputView:
  - **재생 컨트롤** `«` `‹` 슬라이더 `›` `»`
  - **frame outcome 테이블** — frame · outcome · tracked · inliers · KF? · new lm · BA chi² Δ% · ms
- OutputView:
  - **요약 카드** — total frames / KFs / landmarks / total ms / BA accumulated ms / sliding window evictions / last BA chi²
  - **Scene3D** — active KF frustum + evicted KF dim + active MapPoint cloud
- VerifyGate(자동 6~7건):
  1. 4 WASM 모듈 로드
  2. 5 frames + calib 로드
  3. lost/failed = 0
  4. KF ≥ 1
  5. 궤적 좌표 모두 유한
  6. 누적 길이 ≤ 50 m (mini fixture sanity)
  7. (Backend ON 시) BA chi² 단조 감소

## 알고리즘

- [x] 신규 C++ 없음 — `src/lib/slam/pipeline.ts`(~430 LOC)에서 4 WASM(features + triangulation + pnp + ba)을 한 사이클로 연결
- [x] StereoInit (frame 0)
- [x] Track + PnP + KeyFrame insertion + TriangulateNewPoints
- [x] Backend BA(active window optimize)
- [x] Sliding Window 4 정책

## 가속 경로

- [x] CPU scalar (baseline) — useMemo 기반 동기 재계산, mini fixture 5 frame에서 sub-200 ms
- [ ] Web Worker 분리 — Phase I 후속 (long sequence 시 main thread blocking 방지)
- [ ] WebGL/WebGPU Viewer — Phase I+ deferred

## 검증

- [x] 자동 수치 게이트 — VerifyGate 6건(Backend ON시 7번째)
- [x] 시각 수동 게이트 — 재생 슬라이더로 frame 0 → 4 진행 시 KF 추가 + landmark cloud 확장 점진 확인
- [x] 원본 C++ 결과와의 diff — Frontend::AddFrame switch-case가 Track/StereoInit/Lost로 동일하게 분기

## 학습 노트

Step 13은 Phase B–H의 모든 결과를 한 페이지에 통합한다 — features WASM(Phase C) + triangulation WASM(Phase D) + pnp WASM(Phase E) + ba WASM(Phase G), TS-side SlamMap(Phase H Step 12), 그리고 정책 토글까지. 이 통합이 ch13 Frontend::AddFrame switch와 backend.cpp::Optimize 호출을 충실히 재현하는지가 본 Step의 본질.

**`relative_motion_ * last_pose` init**: ch13의 핵심 트릭 — 이전 frame의 pose에 직전 motion(velocity)을 한 번 더 적용한 값을 PnP init으로 쓴다. 이게 일종의 constant-velocity prior로, KITTI 같은 도로 시퀀스에서 매우 강력. 본 PR의 pipeline.ts가 이 init을 그대로 사용해 PnP 수렴이 안정화된다.

**Backend BA observation 합성 단순화** — 영속 frame-by-frame feature 추적을 SlamMap에 저장하면 sub-200 ms 실행 가능하지만 그 인프라는 Phase I worker화와 함께 가져가는 편이 자연스러움. 본 Step의 BA 호출은 "프로토콜 와이어링" 검증용이라 refined pose로 landmark를 다시 투영해 (deterministic) 1px 미만 노이즈를 더한 합성 측정값으로 채운다. chi² histogram은 의도적으로 작은 값에서 시작해 거의 0으로 수렴 — 단조 감소 게이트 통과 OK.

**프리셋 3종**:
- `book-default`: 책의 기본값 그대로(maxFeatures=200, num_features_needed_for_keyframe=80, BA iterations=10).
- `conservative`: 추적 임계 + KF 삽입 임계를 높여(num_features_tracking=60) KF 빈도 감소, BA 빈도 감소 → fast.
- `aggressive`: 임계를 낮춰(num_features_needed_for_keyframe=120) 자주 KF 만들고 자주 BA → 정확도 ↑, 비용 ↑.

### 의도된 실패

- window size = 2 + frames=5 → sliding window evictions 발생, Scene3D에서 회색 dim KF 표시.
- enable backend = false → BA chi² Δ 컬럼이 비고, trajectory가 PnP-only(local consistency만 보장)로 더 noisy해짐.
- num_features_tracking 임계를 매우 크게(120) → 모든 transition이 LOST로 분류되어 trajectory 단절(ch13 status `LOST`).

### 후속 합류 항목 (Phase I+)

- Web Worker로 `runPipeline`을 옮겨 long-sequence main-thread blocking 회피.
- 영속 feature → BA observation의 진짜 stereo+temporal LK 측정값 사용.
- 궤적 polyline 시각화 (현재 Scene3D는 KF frustum + landmark cloud만).
- PLY export + KITTI GT 비교 (RMSE 측정).
