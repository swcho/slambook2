# Step 09 — Keyframe Decision

## 학습 목표

- [x] 키프레임 정책이 정확도/연산비용에 주는 영향
- [x] inlier-count vs motion vs hybrid 정책의 발화 패턴 차이
- [x] long sequence 통계(KF 5~15%)와 mini fixture의 학습용 게이트 차이

## C++ 원본 매핑

- 파일: `ch13/src/frontend.cpp::Frontend::InsertKeyframe`
- 핵심: `tracking_inliers_ < num_features_needed_for_keyframe_(80)`이면 KF 삽입.
- 본 Step은 책의 단일 정책(inlier count) 외에 motion / hybrid 두 정책을 추가 토글.

## UI

- ParamPanel:
  - policy picker (inlier-count(ch13) / motion / hybrid)
  - inlier threshold(50..150) · motion threshold ‖t‖ + rotation deg
  - covariance 정책 = disabled chip (BA의 landmark uncertainty 필요 → Phase G 의존)
- Input View: 4 transition(0→1, 1→2, 2→3, 3→4)에 대해 Step 7+8 자동 실행
- Output View:
  - **tracking_inliers 막대차트** — transition별 inlier 수 + 임계 점선
  - **결정 표** — transition / inliers / ‖t‖ / rotation° / KF? + reason
- VerifyGate(자동 4건): 3 WASM · 4 frames+calib · 4/4 PnP 수렴 · ≥1 KF 트리거

## 알고리즘

- [x] 신규 C++ 없음 — Step 7/8 파이프라인 재사용
- [x] inlier-count (원본 ch13)
- [x] motion-based — `‖t‖` + rotation 임계 OR
- [x] hybrid — inlier OR motion
- [ ] covariance-based — Phase G+ deferred (BA dependency)

## 가속 경로

- [ ] 가속 적용 없음 (정책 함수)

## 검증

- [x] 자동 수치 게이트 — VerifyGate 4건. PLAN §3의 "KF 5~15%"는 long-sequence 기준이라 mini fixture에서는 ≥1 KF로 완화
- [x] 시각 수동 게이트 — 정책 토글 시 tracking_inliers 막대차트의 임계 점선이 변하고 결정 표의 KF? 컬럼이 즉시 갱신
- [x] 원본 C++ 결과와의 diff — inlier-count 정책은 InsertKeyframe 조건과 동일

## 학습 노트

키프레임 정책은 SLAM의 "어디까지 무거운 작업(triangulation, BA)을 미룰 것인가"를 결정한다. 너무 자주 KF를 삽입하면 BA가 매번 호출되어 실시간성을 잃고, 너무 드물면 추적 실패 시 재초기화 비용이 커진다.

**ch13의 inlier-count 정책**은 단순하지만 효과적 — 추적이 약해지는 순간이 곧 새 KF가 필요한 순간이라는 직관. 그러나 KITTI 같은 long sequence에서는 빠른 회전이나 큰 motion 시점에도 inlier가 일시적으로 떨어져 KF가 너무 자주 삽입될 수 있다. **motion 정책**은 cumulative motion이 충분히 누적되면 inlier 수와 무관하게 KF를 만들어 spatial diversity를 보장. **hybrid**는 둘의 OR로 둘 다 만족 못 하면 KF 안 만듦.

mini fixture는 5 frame → transition 4개 → KF 비율 0/25/50/75/100%로 양자화. 정확한 5~15% 통계는 long sequence에서만 의미. 본 Step은 정책 차이를 **시각적으로 학습**하는 자리이고, 실 평가는 Step 13의 trajectory length sanity로 위임.

### 의도된 실패

- inlier threshold를 250으로 → 모든 transition이 KF가 됨 (overkill).
- motion threshold를 매우 작게(0.001 m / 0.01°) → 실제 정지 상태에서도 KF 발화.
- hybrid + inlier 250 + motion 0.001 → 매 transition KF, BA 폭주(Step 13에서 관찰).
