# ch13-wasm 진행 상태 (PROGRESS)

본 파일은 `PLAN.md`에 기술된 작업의 **진행 상태·결정·블로커**를 기록한다.  
각 Phase/Step 종료 시 이 파일을 반드시 갱신하고 커밋해야 새 세션이 상태를 복원할 수 있다.

- **시작일**: 2026-04-17
- **최근 갱신**: 2026-04-18
- **현재 진행 중**: — (Phase A 헤드리스 검증 완료. **DOM 수동 관측만 사용자 대기**, 이후 세션은 **Phase A+ — g2o/CXSparse WASM 스파이크**부터 시작)

---

## 빠른 상태 요약

| 영역 | 상태 |
|------|------|
| 프로젝트 스캐폴딩 (`ch13-wasm/` 생성) | ✅ 완료 |
| Emscripten 환경 | ✅ Homebrew `emscripten 5.0.6` (아래 주의사항 참고) |
| hello_world WASM E2E | 🟡 Node 스모크 검증(`greet`/`add` 수치 일치) + `npm run dev` 헤드리스 검증(헤더·MIME·SPA fallback) 완료 — 브라우저 내 DOM 렌더링은 사용자 육안 확인 대기 |
| dev 서버 헤드리스 검증 | ✅ 2026-04-18 완료 (아래 "Phase A 검증 기록" 참조) |
| g2o WASM 스파이크 (Phase A+) | ⬜ 미수행 — **결정 전** |
| 현재 브랜치 | `ex` |
| 마지막 커밋 | `890a11c` (fix: rerendering error) → 본 PROGRESS 갱신 커밋 예정 |

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

- **브라우저 육안 관측 남음(비블로커)**: HTTP 계층은 2026-04-18 헤드리스로 검증됨(위 표 참조). 다만 React 마운트 + WASM streaming fetch + `locateFile` 해석은 브라우저 JS 엔진에서만 재현 가능 — Phase A+ 세션 시작 시 `npm run dev` 후 `http://127.0.0.1:5173/`를 열어 Home "WASM hello_world" 필드가 실제로 `hello from wasm, ch13-wasm!`으로 채워지는지 먼저 확인. 실패 시 A+ 진입 전에 수정.
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
- [x] **헤드리스 검증(2026-04-18)**: `npm run dev`로 Vite 띄운 뒤 `curl`로 HTTP 계층 확인 — `/`와 `/wasm/myslam_hello.baseline.{js,wasm}` 모두 `COOP=same-origin`·`COEP=require-corp` 헤더 반영, `.wasm`은 `application/wasm` MIME로 서빙됨, SPA fallback으로 13개 `/step/<slug>`와 `/step/nonexistent`까지 `200 text/html`로 index.html 반환 확인
- [ ] **사용자 육안 관측(남음)**: `npm run dev` 후 Chrome/Edge 121+에서 `http://127.0.0.1:5173/`를 열어 ① Home의 "WASM hello_world" 항목이 `hello from wasm, ch13-wasm!` 문자열로 바뀌는지(= React 마운트 + WASM streaming fetch + `locateFile` 성공), ② 사이드바 13개 Step 링크가 흰/회색 경계 없이 전환되는지 확인. 헤드리스로는 브라우저 JS 엔진 경로를 재현할 수 없어 사용자 대기로 남겨 둔다

### Phase A 검증 기록 (2026-04-18, 헤드리스)

| 항목 | 관측값 |
|------|--------|
| `GET /` | `200 text/html`, `COOP=same-origin`, `COEP=require-corp` |
| `GET /wasm/myslam_hello.baseline.wasm` | `200 application/wasm` (13675 B), COEP/COOP 동일 반영 |
| `GET /wasm/myslam_hello.baseline.js` | `200 text/javascript`, COEP/COOP 동일 반영 |
| `GET /step/<slug>` × 13 (dataset~full-pipeline) | 모두 `200 text/html` — SPA fallback 정상 |
| `GET /step/nonexistent` | `200 text/html` (router의 `path: '*'` → Navigate `/`로 클라이언트 리다이렉트될 것으로 기대) |
| `GET /src/main.tsx`, `/src/wasm/loader.ts` | `200 text/javascript` — Vite 모듈 그래프 정상 |
| `index.html` 내용 | `<div id="root">` + `/src/main.tsx` 모듈 로드 태그 포함 |

---

## 갱신 규칙

1. **Phase/Step 착수 시**: "현재 진행 중" 필드를 갱신.
2. **완료 시**: 해당 체크박스 ✅, "검증 게이트 통과"에 통과 조건 요약을 기록.
3. **결정이 발생하면**: "주요 결정 로그"에 append-only로 한 줄 추가.
4. **블로커 발생**: "현재 블로커"에 원인·시도한 해결책·다음 행동을 기록.
5. **세션 종료 시**: 반드시 이 파일 갱신 후 커밋. 커밋 메시지 예: `progress: finish Phase A scaffolding`.
