# Step 12 — Map Management (Sliding Window)

## 학습 목표

- [x] 영속 Map 그래프(KeyFrame + MapPoint + 관측)의 데이터 모델
- [x] 키프레임 제거 정책 4종 비교 — 중복 우선 / FIFO / covisibility / distance-only
- [x] cleanMap의 orphan landmark drop 동작

## C++ 원본 매핑

- 파일: `ch13/src/map.cpp::Map::RemoveOldKeyframe` + `Map::CleanMap`
- 핵심: 활성 KF가 `num_active_keyframes_(7)`을 초과하면 (a) 매우 가까운 KF가 있으면 그 중 가장 가까운 것 제거(중복 우선), (b) 없으면 가장 먼 것 제거(다양성 유지). 본 Step은 정책을 4종으로 확장해 비교.

## UI

- ParamPanel:
  - num_active_keyframes slider(2..stream-1)
  - policy picker: ch13-default / FIFO / covisibility / distance-only
  - duplicate threshold slider (ch13-default 활성 시만)
  - stream synthesis: forward A 갯수 / duplicates 갯수 / forward B 갯수
- Input View: 합성 10-KF 스트림 정의 카드
- Output View:
  - **정책 비교 표** — 4 정책 동시 실행, evictions / landmarks dropped / final active KFs / final active LMs / pose spread variance
  - **insertion + eviction 로그** — step별 inserted KF / active KFs after / eviction reason + landmarks removed
  - **Scene3D** — active KF frustum(시안) + evicted KF frustum(회색 dim) + active landmark(노랑) + inactive landmark(회색)
- VerifyGate(자동 5건): stream ≥ window+1 · 활성 정책이 ≥ 1 KF evict · evict 횟수 = total−window · cleanMap 후 active landmark > 0 · 4 정책 모두 NaN 없이 종료

## 알고리즘

- [x] 신규 C++ 없음 — TS 영속 모델 `src/lib/slam/{se3.ts, map.ts}` (~525 LOC)
- [x] ch13-default — 중복 분기 → 최원거리 fallback
- [x] FIFO — 시간순
- [x] covisibility — 공통 landmark 적은 KF부터
- [x] distance-only — 항상 closest (diversity 붕괴 학습 케이스)

## 가속 경로

- [ ] 가속 적용 없음 (그래프 트래버설)

## 검증

- [x] 자동 수치 게이트 — `verify_slam.ts` 12건 통과 (se3 log-norm 3 + 정책 evict 패턴 4 + final-active 검증 2 + cleanMap orphan 1 + pose spread 2)
- [x] 시각 수동 게이트 — 정책 토글 시 비교 표의 evictions 컬럼 변화 + Scene3D에서 active vs inactive 색상 전환
- [x] 원본 C++ 결과와의 diff — ch13-default 정책이 RemoveOldKeyframe + CleanMap 로직과 동일

## 학습 노트

영속 Map 그래프는 SLAM의 **stateful core**. KeyFrame은 시간축의 anchor, MapPoint는 공간축의 anchor, 둘 사이의 Feature 관측이 그래프 엣지. 이 그래프가 무한히 자라지 않도록 sliding window를 둔다.

**TS로 둔 이유** — ch13의 Map/MapPoint/KeyFrame은 영속 그래프 + 관측 back-pointer만 다루는 데이터 컨테이너다. 수치 코어는 이미 g2o(BA) / Eigen(triangulation) / OpenCV(features) WASM에 들어가 있고, 컨테이너 자체는 cross-cutting concern이라 C++로 옮길 동기가 적다. WASM heap에 영속 그래프를 두면 React 측 sync boilerplate가 모든 페이지에서 반복되고 `.delete()`/weak_ptr 메모리 위생 부담도 늘어난다. TS class로 두면 useMemo로 reactive하게 새로 빌드할 수 있고, **본 Step의 4 정책 동시 비교**(같은 stream으로 4번 새 SlamMap을 빌드)도 자연스럽다.

**합성 10-KF 스트림 채택 이유** — KITTI mini 5 frame은 모두 14-px forward shift만 있어 모든 KF가 ‖log‖ ≪ 0.2 m 안에 들어가 ch13 default가 항상 closest 분기로만 동작한다. 4 정책의 차이가 사라져 학습 가치 손실. 합성 스트림 `forward A 5 → duplicate cluster 2 → forward B 3`은 의도된 패턴으로 ch13 default(중복→다양성 전환), FIFO(시간순), covisibility(공통 landmark 부족), distance-only(diversity 붕괴) 4가지 행동을 한 페이지에서 비교 가능.

**distance-only가 학습 케이스인 이유** — 항상 가장 가까운 KF를 제거하면 spatial diversity가 무너져 evict 패턴이 `[3, 4, 5, 6, 7, 8]`처럼 forward 클러스터만 남기고 시작점들을 모두 보존하지 못한다. 반면 ch13-default는 중복 분기에서 `[0, 4, 5]`(중복 우선) → `[1, 2, 3]`(다양성 분기) 전환이 일어나 active set이 시작/중간/끝의 3-구간 sample을 유지.

**game spread variance 게이트는 long sequence 기준**이라 mini fixture에서는 ≥1 evict + 4 정책 NaN-free로 단순화. 실 시퀀스 게이트는 Step 13의 trajectory length sanity로 위임.

### 의도된 실패

- distance-only + forward A=2 + duplicates=4 + forward B=2 → 모든 신규 forward KF가 즉시 evict, diversity 붕괴 시연.
- duplicate threshold를 0.01 m로 줄이면 ch13-default가 중복 분기를 거의 못 잡아 항상 farthest로 동작.
- num_active_keyframes = 1 → 매 step마다 evict, active landmark가 거의 모두 orphan으로 drop.
