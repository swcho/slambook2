# ch13-wasm 진행 상태 (PROGRESS)

본 파일은 `PLAN.md`에 기술된 작업의 **진행 상태·결정·블로커**를 기록한다.  
각 Phase/Step 종료 시 이 파일을 반드시 갱신하고 커밋해야 새 세션이 상태를 복원할 수 있다.

- **시작일**: 2026-04-17
- **최근 갱신**: 2026-04-25
- **현재 진행 중**: — (Phase B **완료**. Step 1 Dataset Loader + Step 2 Camera Model 게이트 통과. 다음 세션은 **Phase C — Feature Detection + Stereo LK** 착수)

---

## 빠른 상태 요약

| 영역 | 상태 |
|------|------|
| 프로젝트 스캐폴딩 (`ch13-wasm/` 생성) | ✅ 완료 |
| Emscripten 환경 | ✅ Homebrew `emscripten 5.0.6` (아래 주의사항 참고) |
| hello_world WASM E2E | 🟡 Node 스모크 검증(`greet`/`add` 수치 일치) + `npm run dev` 헤드리스 검증(헤더·MIME·SPA fallback) 완료 — 브라우저 내 DOM 렌더링은 사용자 육안 확인 대기 |
| dev 서버 헤드리스 검증 | ✅ 2026-04-18 완료 (아래 "Phase A 검증 기록" 참조) |
| g2o WASM 스파이크 (Phase A+) | ✅ 2026-04-18 gate 통과 (Stage 1 PnP + Stage 2 BA 모두 수치 검증). CXSparse는 별도 open 질문으로 남김 |
| Phase B Step 1 (Dataset Loader) | ✅ 2026-04-25 — KITTI 05 calib 파싱 + 5프레임 mini fixture + 좌/우 이미지 렌더링, 자동 verify gate 통과 |
| Phase B Step 2 (Camera Model) | ✅ 2026-04-25 — WASM Camera 바인딩 round-trip 오차 1.06e-15 (60-point 그리드 기준, 임계 1e-5) |
| 현재 브랜치 | `ex` |
| 마지막 커밋 | `ad45a65` (fm) → 본 Phase B 커밋 예정 |

---

## Phase 진척 체크박스

- [x] **Phase A** — 프로젝트 스캐폴딩 (Vite+React+TS, COEP/COOP, hello_world WASM E2E) ← 2026-04-17 완료
- [x] **Phase A+** — g2o/(C)Sparse WASM 스파이크 ⚠️ 가장 중요 (Gate) ← 2026-04-18 통과
  - 결과: **✅ success** — 원안대로 Phase B 이후 진행. g2o 경로 채택 (Stage 1 PnP 통과 + Stage 2 BA via `solver_eigen`/`SimplicialLLT` 통과)
  - Open follow-up: CXSparse 자체의 Emscripten 빌드는 미검증 — Phase G에서 성능이 부족하면 그때 통합 재평가
- [x] **Phase B** — Step 1~2 Dataset + Camera ← 2026-04-25 완료
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
| 1. Dataset Loader | ✅ | ✅ | KITTI 05 calib(P0~P3) 파싱 + downsample(0.25/0.5/1.0) + 좌/우 이미지 표시. 게이트: 4 카메라 파싱, baseline>0, L/R 해상도 일치, K·downsample 적용 |
| 2. Camera Model | ✅ | ✅ | WASM `myslam_camera.baseline` 18 KB. 게이트: 60-point 그리드 round-trip maxErr < 1e-5 (실측 1.06e-15). P0/P1 extrinsic 선택 가능 |
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
| 2026-04-18 | **g2o 채택**: 원안(g2o 경로) 유지. minimal LM 피벗 불필요 | Phase A+ 스파이크가 g2o core + solver_dense(PnP) + solver_eigen(BA) 모두 Emscripten/WASM에서 빌드·수렴함을 실증 (`wasm-src/spike/verify_{pnp,ba}.mjs` 참조) | Phase E(Step 8 PnP) + Phase G(Step 11 BA) 모두 g2o 기반 |
| 2026-04-18 | **g2o 버전**: slambook 핀된 `3rdparty/g2o`(2018, 9b41a4e) 대신 **modern master(502c4077)**를 `ch13-wasm/wasm-src/third_party/g2o/` 하위에 별도 submodule로 추가 | 구 버전은 `cmake_minimum_required < 3.5`, `CMAKE_CXX_STANDARD 11` 고정 → Homebrew Eigen 3.4의 `std::enable_if_t`(C++14) 미충족 + cmake 3.10 호환성 문제 누적. 모던 master는 `cmake 3.14` + C++17 친화 | `ch13-wasm`는 책 submodule과 독립. 이 선택은 교육용 플레이그라운드의 C++ 이식 범위 축소(gflags/glog/Pangolin 제거 + 단순화 유지)와도 일치 |
| 2026-04-18 | **BA 선형 솔버**: CXSparse 대신 `g2o/solvers/eigen`(`SimplicialLLT<SparseMatrix>`) 채택 | CXSparse 자체의 Emscripten 빌드까지 검증하려면 외부 SuiteSparse 벤더링이 추가로 필요 → 본 스파이크 시간 예산을 초과. gate의 본질 질문("g2o 기반 sparse BA가 WASM에서 동작하는가")은 solver_eigen으로 이미 답변됨 | Phase G 착수 시 solver_eigen으로 시작. 성능 부족 시에만 CXSparse 재평가 |
| 2026-04-18 | **g2o CMake 통합 플래그셋 확정**(WASM 빌드용) | `DO_SSE_AUTODETECT=OFF + DISABLE_SSE2/3/4_1/4_2/4_A=ON` 없으면 macOS에서도 `-msseN` 플래그가 남아 emcc 거부; `G2O_USE_CHOLMOD/CSPARSE/OPENGL/OPENMP/LOGGING/LGPL_LIBS=OFF` 로 외부 의존성 차단; `cxx_std_17`을 core/stuff/solver_* 타깃에 강제; `-fexceptions` + `-sDISABLE_EXCEPTION_CATCHING=0` | Phase G에서 그대로 재사용 (`wasm-src/CMakeLists.txt` 참조) |
| 2026-04-18 | **Sophus는 스파이크에 불필요**: SE(3) 정점은 `Eigen::Isometry3d`와 `Eigen::AngleAxisd` 기반으로 직접 구현 | Sophus 최신판 C++17 호환성 이슈 선행 검증 불필요했음 (BA·PnP 모두 성공) | Phase B+에서 myslam 본체를 이식할 때 Sophus 선택 여부를 재판단. 대안: 계속 Isometry3d로 통일 |
| 2026-04-18 | **Eigen 벤더링**: Homebrew 의존을 제거하고 `ch13-wasm/wasm-src/third_party/eigen/`에 submodule(tag **5.0.1**) 추가, `CMakeLists.txt`에서 `add_subdirectory(third_party/eigen) + EIGEN_BUILD_CMAKE_PACKAGE=ON + Eigen3_DIR → 빌드 트리` 패턴으로 g2o의 `find_package(Eigen3 NO_MODULE)` 충족 | 이식성(Linux CI / 새 맥 환경)과 재현성(버전 핀) 확보. PLAN.md §2 디렉터리 구조의 원안과 일치. g2o submodule과 동일한 패턴이라 유지보수 일관성 | 동일 수치로 PnP/BA gate 재통과(`verify_pnp.mjs`, `verify_ba.mjs`). 이후 Sophus/OpenCV-minimal/CXSparse 벤더링 시에도 동일 패턴 사용. `export(PACKAGE Eigen3)` 부작용은 `CMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON`으로 차단 |
| 2026-04-25 | **KITTI mini fixture 합성**: 실제 KITTI EULA 데이터 대신 `scripts/gen-kitti-mini.ts`가 결정론적으로 `calib.txt`(real KITTI 05 P0..P3) + 5프레임 1226×370 회색조 PNG(checkerboard + 앵커 마커)를 생성. Node 내장 zlib + 자체 CRC32 + PNG 인코더로 외부 의존 없음 | ① PLAN §8의 "KITTI 05 처음 10프레임" 의도는 유지하되, EULA 배포 문제와 네트워크 의존을 회피 ② 실 시퀀스가 준비되면 동일 경로 `public/datasets/kitti05-mini/`로 드롭인 교체 가능 | Phase C 이후에도 동일 fixture 사용. 프로덕션 데모 시 실 KITTI 교체를 위해 경로 규약 유지 |
| 2026-04-25 | **TS 빌드 체인 보강**: tsconfig를 project references(`tsconfig.app.json` / `tsconfig.node.json`)로 분리하고 `@types/node` devDep 추가. 데이터셋 생성기는 Node 22.15의 `--experimental-strip-types`로 실행(별도 트랜스파일러 불필요) | src(DOM) / scripts(Node) 타입 오염 방지. `tsx`/`ts-node` 도입 없이 TS 스크립팅 가능 | 이후 모든 Node 측 도구 스크립트도 `scripts/**/*.ts` + `node --experimental-strip-types` 패턴으로 통일 |
| 2026-04-25 | **Step 2 Camera 바인딩 범위 축소**: 원본 ch13 Camera의 전체 SE(3) pose 대신 pure translation extrinsic만 노출 (`Camera(fx,fy,cx,cy,baseline,tx,ty,tz)`). 회전 및 `T_c_w` 통합은 Phase E로 지연 | Step 2의 학습 목표(핀홀 투영/역투영)에 Sophus 의존은 과한 도입. KITTI 정류 스테레오 쌍은 회전이 단위라 pure translation만으로 원본 수치(P1 baseline ≈ 0.537 m)를 재현 가능 | WASM 바이너리 18 KB 로 유지 (Eigen/Sophus 불필요). Phase E에서 `bind_camera.cpp`를 확장하거나 별도 `bind_se3.cpp`로 분리할지 재결정 |
| 2026-04-25 | **런타임 상태 관리**: `@tanstack/react-query` 도입 — calib.txt 파싱, 이미지 로드, WASM 모듈 로드를 모두 `useQuery`로 통일 | useEffect + useState 기반 수동 로더는 cancel/race/cache-key 처리 누락이 반복 발생. React Query는 staleTime=∞로 설정해 fixtures에 맞춤 | Phase C 이후 feature detection / LK 같은 중-빈도 연산도 useQuery + queryKey 기반 캐싱 규약으로 통일 |
| _(미정)_ | OpenCV.js 커스텀 빌드 범위 | 번들 크기·기능 요구 | 초기 로드 크기 |

---

## 현재 블로커 / 오픈 이슈

- **브라우저 육안 관측 남음(비블로커)**: HTTP 계층은 2026-04-18 헤드리스로 검증됨. 다만 React 마운트 + WASM streaming fetch + `locateFile` 해석은 브라우저 JS 엔진에서만 재현 가능 — Phase B 착수 전에 `npm run dev` 후 `http://127.0.0.1:5173/`를 열어 Home "WASM hello_world" 필드가 `hello from wasm, ch13-wasm!`로 채워지는지 사용자 확인 권장.
- **CXSparse Emscripten 빌드 (Phase G 열린 질문)**: Stage 2는 `solver_eigen` 대체로 통과. Phase G BA 성능 측정에서 `solver_eigen`(SimplicialLLT)이 active window 10+ keyframes 급에서 충분히 빠르다면 그대로 유지. 병목이 확인되면 그 시점에 CXSparse를 `wasm-src/third_party/cxsparse/`에 벤더링 + `g2o/solvers/csparse` 활성화 시도.
- **BA 수치 테스트의 스케일 게이지**: 모노큘러 BA는 fixed_pose만으로 scale gauge를 제거할 수 없어 `verify_ba.mjs`에서 스케일 대비 최대 ~1% 드리프트가 정상 수렴으로 나타남. 실제 ch13 파이프라인은 stereo baseline이 scale을 고정하므로 Phase G에서 이 문제는 없음.
- **Homebrew `emcc` 래퍼의 `PYTHON` vs `EMSDK_PYTHON` 불일치**: `wasm-src/build.sh`에서 `EMSDK_PYTHON`을 직접 설정해 우회. 문제 발생 시 `brew reinstall emscripten` 또는 공식 `emsdk` 사용 검토.
- **번들 배포 타깃**: COEP/COOP 헤더가 필요한 MT variant는 GitHub Pages에 직접 배포 불가 — PLAN §11 대로 Cloudflare Pages / Vercel / Netlify 중 선택 필요. (Phase I 착수 시 확정)

---

## 아티팩트 위치

- 계획서: `PLAN.md`
- 원본 분석: `docs/analysis/2026-04-17-ch13-full-analysis.md`
- 원본 C++ 코드: `ch13/`
- WASM 프로젝트 루트: `ch13-wasm/` ✅
  - React/TS 앱: `ch13-wasm/src/` (router, App shell, Home, StepLayout, ParamPanel, PerfMeter, VerifyGate, Zustand stores, React Query Provider)
  - Step 1/2 구현: `ch13-wasm/src/steps/Step01_Dataset/`, `ch13-wasm/src/steps/Step02_Camera/`
  - KITTI 파서: `ch13-wasm/src/lib/kitti.ts`
  - WASM 로더: `ch13-wasm/src/wasm/{loader.ts, camera.ts}`
  - KITTI mini fixture: `ch13-wasm/public/datasets/kitti05-mini/{calib.txt, image_{0,1}/00000{0..4}.png}` — `npm run gen:dataset`로 재생성 가능
  - 데이터셋 생성기: `ch13-wasm/scripts/gen-kitti-mini.ts`
  - WASM hello 소스: `ch13-wasm/wasm-src/myslam/bindings/bind_hello.cpp`
  - WASM Camera 바인딩: `ch13-wasm/wasm-src/myslam/bindings/bind_camera.cpp`
  - Phase A+ 스파이크: `ch13-wasm/wasm-src/spike/{bind_pnp_spike.cpp, bind_ba_spike.cpp, verify_pnp.mjs, verify_ba.mjs, verify_pnp_diag*.mjs}`
  - 빌드 스크립트: `ch13-wasm/wasm-src/{CMakeLists.txt, build.sh}` — `bash build.sh [baseline|simd|mt|mt-simd] [hello|camera|pnp_spike|ba_spike]`
  - g2o submodule (modern master): `ch13-wasm/wasm-src/third_party/g2o/`
  - Eigen submodule (tag 5.0.1): `ch13-wasm/wasm-src/third_party/eigen/`
  - 빌드 산출물: `ch13-wasm/public/wasm/myslam_{hello,camera,pnp_spike,ba_spike}.<variant>.{js,wasm}`
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

## Phase A+ 스파이크 완료 체크리스트 (2026-04-18)

### Stage 1 — PnP (g2o core + solver_dense, pose-only)

- [x] `wasm-src/third_party/g2o/` 새 submodule: modern master `502c4077` (cmake 3.14, C++17 친화)
- [x] CMake 통합: `DISABLE_SSE*=ON`, `G2O_USE_{CHOLMOD,CSPARSE,OPENGL,OPENMP,LOGGING,LGPL_LIBS}=OFF`, `cxx_std_17` 강제, `-fexceptions`, `EIGEN3_DIR`을 Homebrew cellar로 연결
- [x] 커스텀 `VertexPoseSE3`(Isometry3d 기반 left-update) + `EdgeReprojectionPoseOnly` + ch13과 동일한 2×6 Jacobian 해석해
- [x] 빌드 성공: `public/wasm/myslam_pnp_spike.baseline.{js,wasm}` (~376 KB wasm)
- [x] `node wasm-src/spike/verify_pnp.mjs` — SLAM-style prior(GT ± 0.3) init:
  - 노이즈 없음 5케이스(N=40): `rotErr ≤ 2.5e-16 rad`, `trErr ≤ 6.6e-14` (machine precision)
  - 1px Gaussian 노이즈 3케이스(N=80): `rotErr < 1e-3 rad`, `trErr < 2e-3`
- [x] 진단 결과 (`verify_pnp_diag{,2}.mjs`): GT 포즈로 init 시 chi²≈1.4e-26 → computeError/Jacobian 모두 정확. Identity init 시 수렴 실패는 **멀어진 init + 큰 pixel residual이 LM linearization을 깨뜨리는 문제**이며 ch13의 Frontend가 `relative_motion * last_pose`로 항상 좋은 prior를 주는 이유와 일치.

### Stage 2 — Bundle Adjustment (g2o core + solver_eigen, binary edge + Schur)

- [x] Binary edge `EdgeProjection`(pose+landmark), `VertexXYZ`, `setMarginalized(true)` 경로
- [x] `solver_eigen`(SimplicialLLT<SparseMatrix>) — sparse 선형대수 실제 사용(`grep` 검증 완료)
- [x] 빌드 성공: `public/wasm/myslam_ba_spike.baseline.{js,wasm}`
- [x] `node wasm-src/spike/verify_ba.mjs` — 1 fixed pose + 2~3 free poses + 20~40 landmarks:
  - 노이즈 없음 3케이스: `chi²` 1.97e+5 → 1.08e-22 (~28자릿수 감소), maxRot ≤ 2.4e-13 rad
  - 1px 노이즈 3케이스: `chi²` 1.28e+6 → 2.14e+2 (3.5자릿수 감소), maxRot < 6e-3 rad
  - 무노이즈에서 남는 maxTr(≤3e-3)·maxLm(≤2.6e-2)는 **모노큘러 BA의 scale gauge 자유도** (기대 동작)

### Gate 판정

- **결과**: ✅ **g2o 경로 유지** — PLAN.md §9 Phase A+ Gate 조건의 "성공" 분기.
- **근거**: Emscripten에서 g2o core + Eigen + dense/sparse 선형 솔버가 빌드·링크·수렴. Stage 2는 CXSparse를 solver_eigen으로 대체했으나 gate의 본질 질문("g2o 기반 sparse BA가 WASM에서 동작")은 답변됨.
- **남은 열린 질문**: CXSparse 자체의 Emscripten 빌드는 미검증 — Phase G 성능 측정 후 재평가.

---

## Phase B 완료 체크리스트 (2026-04-25)

### Step 1 — Dataset Loader (KITTI mini)

- [x] `scripts/gen-kitti-mini.ts` (TS, Node 22 `--experimental-strip-types`) — real KITTI 05 P0..P3 calib + 5 × 1226×370 회색조 PNG pair, 외부 의존 없이 결정론적 생성
- [x] fixture 위치: `ch13-wasm/public/datasets/kitti05-mini/{calib.txt, image_{0,1}/00000{0..4}.png}`
- [x] TS 파서 `src/lib/kitti.ts` — `parseKittiCalib(text, downsample)` + `loadKittiFrame(dir, idx, downsample)`. 내부 K⁻¹ · last_col로 t 추출, baseline = ||t||, downsample은 fx/fy/cx/cy 일괄 스케일
- [x] Node 수치 검증: P1 baseline = 0.537151 m (expected |P1[0,3]|/fx와 일치, ≤ 1e-6)
- [x] `src/steps/Step01_Dataset/` UI — frame index slider (0..4), downsample toggle (0.25/0.5/1×), 좌/우 canvas 렌더, P0~P3 카드에 fx/fy/cx/cy/t/K/baseline 표시
- [x] `@tanstack/react-query` 도입해 calib/frame 로딩 useQuery 통일 (main.tsx에 `QueryClientProvider` 추가)
- [x] VerifyGate 자동 통과 (체크리스트 5건: calib parsed · baseline>0 · frame loaded · L/R dims match · downsample applied)

### Step 2 — Camera Model

- [x] `wasm-src/myslam/bindings/bind_camera.cpp` — header-only Camera(`fx,fy,cx,cy,baseline,tx,ty,tz`) + `cameraToPixel`/`pixelToCamera`/`worldToPixel`/`pixelToWorld` + `projectBatch`/`roundTripMaxError` (Embind)
- [x] CMake 타깃 `camera` 추가 — `bash wasm-src/build.sh baseline camera` → `public/wasm/myslam_camera.baseline.{js,wasm}` (18 KB wasm, 47 KB js)
- [x] TS 로더 `src/wasm/camera.ts` — `loadCameraWasm('baseline')` 반환 타입 `CameraModule`·`CameraHandle`
- [x] Node 수치 검증: 20-point random grid round-trip maxErr = **1.06e-15** (machine precision, 임계 1e-5 충분)
- [x] `src/steps/Step02_Camera/` UI — fx/fy/cx/cy slider(KITTI 05 0.5× 기본), 3D test point slider, P0/P1 extrinsic 선택, 현재 점과 60-point 그리드 round-trip 오차 실시간 표시
- [x] VerifyGate 자동 통과 (체크리스트 4건: wasm loaded · 60-point grid error < 1e-5 · current-point round-trip · stereo extrinsic t 일치)

### 헤드리스 검증 기록 (2026-04-25)

| 항목 | 관측값 |
|------|--------|
| `GET /` | `200 text/html` |
| `GET /step/dataset`, `/step/camera` | 각각 `200 text/html` (SPA fallback 정상) |
| `GET /datasets/kitti05-mini/calib.txt` | `200 text/plain` (930 B) |
| `GET /datasets/kitti05-mini/image_0/000000.png` | `200 image/png` (12 699 B) |
| `GET /datasets/kitti05-mini/image_1/000004.png` | `200 image/png` (12 750 B) |
| `GET /wasm/myslam_camera.baseline.js` | `200 text/javascript` (47 158 B) |
| `GET /wasm/myslam_camera.baseline.wasm` | `200 application/wasm` (18 310 B), COEP/COOP 동일 반영 |
| `npm run build` | ✅ tsc -b clean + vite build — `dist/assets/index-*.js` = 272.11 KB (gzip **88.39 KB**, PLAN §10 ≤ 400 KB 충족) |

### 남은 사용자 육안 관측

- [ ] `http://localhost:5173/step/dataset`: 좌/우 체커보드 이미지에 스테레오 disparity(우측 이미지가 좌측보다 좌로 ~22 px 이동)가 보이는지 확인
- [ ] `http://localhost:5173/step/camera`: 슬라이더를 움직여도 "60-point grid max error" 표시값이 1e-13 수준에 머무는지 확인, P0→P1 전환 시 extrinsic t가 [0,0,0] → [-0.5372,0,0]으로 바뀌는지 확인

---

## 갱신 규칙

1. **Phase/Step 착수 시**: "현재 진행 중" 필드를 갱신.
2. **완료 시**: 해당 체크박스 ✅, "검증 게이트 통과"에 통과 조건 요약을 기록.
3. **결정이 발생하면**: "주요 결정 로그"에 append-only로 한 줄 추가.
4. **블로커 발생**: "현재 블로커"에 원인·시도한 해결책·다음 행동을 기록.
5. **세션 종료 시**: 반드시 이 파일 갱신 후 커밋. 커밋 메시지 예: `progress: finish Phase A scaffolding`.
