# ch13-wasm 진행 상태 (PROGRESS)

본 파일은 `PLAN.md`에 기술된 작업의 **진행 상태·결정·블로커**를 기록한다.  
각 Phase/Step 종료 시 이 파일을 반드시 갱신하고 커밋해야 새 세션이 상태를 복원할 수 있다.

- **시작일**: 2026-04-17
- **최근 갱신**: 2026-04-17
- **현재 진행 중**: — (미착수. 다음 세션은 **Phase A**부터 시작)

---

## 빠른 상태 요약

| 영역 | 상태 |
|------|------|
| 프로젝트 스캐폴딩 (`ch13-wasm/` 생성) | ⬜ 미착수 |
| Emscripten 환경 | ⬜ 미설치 |
| g2o WASM 스파이크 (Phase A+) | ⬜ 미수행 — **결정 전** |
| 현재 브랜치 | `ex` (PLAN.md만 작성된 상태) |
| 마지막 커밋 | — |

---

## Phase 진척 체크박스

- [ ] **Phase A** — 프로젝트 스캐폴딩 (Vite+React+TS, COEP/COOP, hello_world WASM E2E)
- [ ] **Phase A+** — g2o/CXSparse WASM 스파이크 ⚠️ 가장 중요 (Gate)
  - 결과: `pending` (성공 → 원안 진행 / 실패 → minimal LM 피벗)
- [ ] **Phase B** — Step 1~2 Dataset + Camera
- [ ] **Phase C** — Step 3~4 Feature Detection + Stereo LK
- [ ] **Phase D** — Step 5~6 Triangulation + Initial Map
- [ ] **Phase E** — Step 7~8 Frame Tracking + PnP
- [ ] **Phase F** — Step 9~10 Keyframe + New MapPoints
- [ ] **Phase G** — Step 11 Bundle Adjustment (난이도 최상)
- [ ] **Phase H** — Step 12~13 Sliding Window + Full Pipeline
- [ ] **Phase I** — a11y·모바일·문서·Playwright 스모크

---

## Step 검증 게이트 통과 현황

| Step | 구현 | 검증 게이트 통과 | 비고 |
|------|------|----------------|------|
| 1. Dataset Loader | ⬜ | ⬜ | |
| 2. Camera Model | ⬜ | ⬜ | |
| 3. Feature Detection | ⬜ | ⬜ | |
| 4. Stereo Matching (LK) | ⬜ | ⬜ | |
| 5. Triangulation | ⬜ | ⬜ | |
| 6. Initial Map | ⬜ | ⬜ | |
| 7. Frame Tracking | ⬜ | ⬜ | |
| 8. Pose Estimation (PnP) | ⬜ | ⬜ | |
| 9. Keyframe Decision | ⬜ | ⬜ | |
| 10. New MapPoints | ⬜ | ⬜ | |
| 11. Bundle Adjustment | ⬜ | ⬜ | |
| 12. Sliding Window | ⬜ | ⬜ | |
| 13. Full Pipeline | ⬜ | ⬜ | |

---

## 주요 결정 로그

> Phase A+ 스파이크 및 이후 발생하는 아키텍처 결정은 여기 append-only로 기록한다.

| 날짜 | 결정 | 근거 | 영향 |
|------|------|------|------|
| 2026-04-17 | PLAN.md 초안 확정 | 초기 설계 | — |
| _(미정)_ | g2o WASM 포팅 채택 여부 | Phase A+ 스파이크 결과 | Step 8·11 구현 경로 |
| _(미정)_ | Sophus 버전 또는 C++17 업그레이드 | Phase A+ 빌드 검증 | 전체 컴파일 플래그 |
| _(미정)_ | OpenCV.js 커스텀 빌드 범위 | 번들 크기·기능 요구 | 초기 로드 크기 |

---

## 현재 블로커 / 오픈 이슈

_(없음 — 시작 전)_

---

## 아티팩트 위치

- 계획서: `PLAN.md`
- 원본 분석: `docs/analysis/2026-04-17-ch13-full-analysis.md`
- 원본 C++ 코드: `ch13/`
- (예정) WASM 프로젝트 루트: `ch13-wasm/`
- (예정) 단계별 학습 노트: `docs/steps/NN-*.md`

---

## 갱신 규칙

1. **Phase/Step 착수 시**: "현재 진행 중" 필드를 갱신.
2. **완료 시**: 해당 체크박스 ✅, "검증 게이트 통과"에 통과 조건 요약을 기록.
3. **결정이 발생하면**: "주요 결정 로그"에 append-only로 한 줄 추가.
4. **블로커 발생**: "현재 블로커"에 원인·시도한 해결책·다음 행동을 기록.
5. **세션 종료 시**: 반드시 이 파일 갱신 후 커밋. 커밋 메시지 예: `progress: finish Phase A scaffolding`.
