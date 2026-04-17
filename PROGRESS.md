# ch13-wasm 진행 상태 (PROGRESS)

본 파일은 `PLAN.md`에 기술된 작업의 **진행 상태·결정·블로커**를 기록한다.  
각 Phase/Step 종료 시 이 파일을 반드시 갱신하고 커밋해야 새 세션이 상태를 복원할 수 있다.

- **시작일**: 2026-04-17
- **최근 갱신**: 2026-04-17
- **현재 진행 중**: — (Phase A 완료. 다음 세션은 **Phase A+ — g2o/CXSparse WASM 스파이크**부터 시작)

---

## 빠른 상태 요약

| 영역 | 상태 |
|------|------|
| 프로젝트 스캐폴딩 (`ch13-wasm/` 생성) | ✅ 완료 |
| Emscripten 환경 | ✅ Homebrew `emscripten 5.0.6` (아래 주의사항 참고) |
| hello_world WASM E2E | 🟡 Node 스모크 검증(`greet`/`add` 수치 일치) — 실제 브라우저 경로(streaming fetch + `locateFile`)는 Phase A+ 세션 시작 시 직접 관측 필요 |
| g2o WASM 스파이크 (Phase A+) | ⬜ 미수행 — **결정 전** |
| 현재 브랜치 | `ex` |
| 마지막 커밋 | `<pending>` (Phase A 커밋 예정) |

---

## Phase 진척 체크박스

- [x] **Phase A** — 프로젝트 스캐폴딩 (Vite+React+TS, COEP/COOP, hello_world WASM E2E) ← 2026-04-17 완료
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
| 2026-04-17 | **Phase A 스캐폴드 채택**: Vite 5 + React 18 + TS 5 + react-router-dom v6 + Zustand 5 | PLAN §1.1/§2 원안 + Node 22 환경에서 동작 확인 | 이후 Phase에서 이 구성 위에 Step UI/WASM 바인딩을 쌓음 |
| 2026-04-17 | **WASM 빌드 체인**: Homebrew `emscripten 5.0.6` + CMake + `--bind` + ES6 모듈(`EXPORT_ES6=1`) | PLAN §1.2, §7.2. Emscripten 5.x가 플래그 상위 호환 | `CH13_WASM_VARIANT`로 baseline/simd/mt/mt-simd 분기 |
| 2026-04-17 | **Emscripten Python 우회**: `EMSDK_PYTHON`을 `python@3.14`로 고정 | Homebrew 래퍼가 `PYTHON` 변수를 쓰는데, 내부 `emcc` 스크립트는 `EMSDK_PYTHON`/`python3`를 우선 참조 → 시스템 Python 3.9로 떨어져 실패 | `wasm-src/build.sh`에서 자동 설정 |
| _(미정)_ | g2o WASM 포팅 채택 여부 | Phase A+ 스파이크 결과 | Step 8·11 구현 경로 |
| _(미정)_ | Sophus 버전 또는 C++17 업그레이드 | Phase A+ 빌드 검증 | 전체 컴파일 플래그 (Phase A의 hello_world는 C++17 기본 채택) |
| _(미정)_ | OpenCV.js 커스텀 빌드 범위 | 번들 크기·기능 요구 | 초기 로드 크기 |

---

## 현재 블로커 / 오픈 이슈

- **브라우저 실시간 관측 미수행**: Phase A 작업은 헤드리스 환경에서 진행되어 실제 Chrome/Safari에서 React 앱 + WASM 동적 import + `locateFile` 경로를 눈으로 확인하지 못했다. Phase A+ 세션 시작 시 `npm run dev` 후 `http://127.0.0.1:5173/`를 먼저 열어 Home의 greeting 문자열과 13개 Step 라우트 전환을 관찰할 것. 문제 발견 시 A+ 작업 전에 수정.
- **Homebrew `emcc` 래퍼의 `PYTHON` vs `EMSDK_PYTHON` 불일치**: `/opt/homebrew/bin/emcc`가 설정하는 `PYTHON` 변수는 실제 `emcc` 스크립트에 전달되지 않아 시스템 Python 3.9로 폴백되어 실패한다. `wasm-src/build.sh`에서 `EMSDK_PYTHON`을 직접 설정해 우회. 문제 발생 시 `brew reinstall emscripten` 또는 공식 `emsdk` 사용 검토.
- **번들 배포 타깃**: COEP/COOP 헤더가 필요한 MT variant는 GitHub Pages에 직접 배포 불가 — PLAN §11 대로 Cloudflare Pages / Vercel / Netlify 중 선택 필요. (Phase I 착수 시 확정)

---

## 아티팩트 위치

- 계획서: `PLAN.md`
- 원본 분석: `docs/analysis/2026-04-17-ch13-full-analysis.md`
- 원본 C++ 코드: `ch13/`
- WASM 프로젝트 루트: `ch13-wasm/` ✅
  - React/TS 앱: `ch13-wasm/src/` (router, App shell, Home, StepLayout, ParamPanel, PerfMeter, VerifyGate, Zustand stores)
  - WASM 소스: `ch13-wasm/wasm-src/myslam/bindings/bind_hello.cpp` (+ `CMakeLists.txt`, `build.sh`)
  - 빌드 산출물: `ch13-wasm/public/wasm/myslam_hello.baseline.{js,wasm}` (Phase A 검증용)
- (예정) 단계별 학습 노트: `docs/steps/NN-*.md`

---

## Phase A 완료 체크리스트

- [x] `ch13-wasm/` 디렉토리 + Vite+React+TS 스캐폴드 (`package.json`, `tsconfig.json`, `vite.config.ts`, `index.html`)
- [x] `vite.config.ts`에서 `dev`·`preview` 헤더 모두 `COOP=same-origin` / `COEP=require-corp`
- [x] `curl -I http://127.0.0.1:5174/`로 헤더 반영 확인
- [x] `curl -I /wasm/myslam_hello.baseline.wasm`로 헤더 반영 확인
- [x] react-router-dom v6: 13개 Step 라우트 + `/` Home 라우트
- [x] Zustand stores: `pipelineStore`(persist with localStorage) / `paramStore` / `benchStore`
- [x] 공통 컴포넌트 skeleton: `StepLayout`, `ParamPanel`, `PerfMeter`, `VerifyGate`
- [x] Emscripten + CMake + `bind_hello.cpp` + `build.sh` — baseline 빌드 성공
- [x] `loadHelloWasm()` 경유 TS 타입, Node에서 `greet("node") → "hello from wasm, node!"`, `add(2,3)=5` 확인 — Node는 `wasmBinary` 주입 경로였음(브라우저의 streaming fetch + `locateFile` 경로와 다름)
- [x] `npx tsc -b` 에러 0건, `vite build` gzip 72KB (PLAN §10 목표 400KB 이하)
- [ ] **남은 관측**: 실제 브라우저(Chrome/Edge 121+)에서 `/` Home 페이지가 `greet("ch13-wasm")`의 결과 문자열을 화면에 출력하는지, `/step/dataset` 등 13개 라우트가 404 없이 전환되는지를 Phase A+ 세션 시작 시 먼저 수동 확인할 것

---

## 갱신 규칙

1. **Phase/Step 착수 시**: "현재 진행 중" 필드를 갱신.
2. **완료 시**: 해당 체크박스 ✅, "검증 게이트 통과"에 통과 조건 요약을 기록.
3. **결정이 발생하면**: "주요 결정 로그"에 append-only로 한 줄 추가.
4. **블로커 발생**: "현재 블로커"에 원인·시도한 해결책·다음 행동을 기록.
5. **세션 종료 시**: 반드시 이 파일 갱신 후 커밋. 커밋 메시지 예: `progress: finish Phase A scaffolding`.
