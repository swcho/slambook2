# ch13-wasm 진행 상태 (PROGRESS)

본 파일은 `PLAN.md`에 기술된 작업의 **진행 상태·결정·블로커**를 기록한다.  
각 Phase/Step 종료 시 이 파일을 반드시 갱신하고 커밋해야 새 세션이 상태를 복원할 수 있다.

- **시작일**: 2026-04-17
- **최근 갱신**: 2026-04-26
- **현재 진행 중**: **Phase D 착수 대기** — Phase C Step 3(Feature Detection) + Step 4(Stereo LK) 본 작업 완료(2026-04-26). 다음: Phase D Step 5(Triangulation) + Step 6(Initial Map). SIMD/MT variant·WebGL/WebGPU 가속 경로는 Phase C+로 deferred.

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
| Phase C 게이트 spike (Q1~Q4) | ✅ 2026-04-26 — OpenCV 4.13.0 분리 빌드 + cv_spike 단일 myslam.wasm 통합 + GFTT 200/200 + LK 100% (mean dx = −14.00 px) |
| Phase C Step 3 (Feature Detection) | ✅ 2026-04-26 — `myslam_features.baseline.wasm` 1.14 MB. GFTT/Harris/FAST/ORB 4개 algo + 마스크 + Step3 UI (AlgoPicker, 슬라이더, 오버레이). 4개 algo 모두 200/200 kps, mask 100×100 hole 0/0 |
| Phase C Step 4 (Stereo LK) | ✅ 2026-04-26 — `bind_features.trackLK` (winSize/maxLevel/maxIter/eps/useInitialFlow). Step4 UI (좌→우 매칭선, 200/200 트랙, mean dx=-21.95 px, mean &#124;dy&#124;=0.00 px on synthetic stereo) |
| 현재 브랜치 | `ex` |
| 마지막 커밋 | `c3d1659` (Phase C 게이트 spike) → 본 작업 커밋 예정 |

---

## Phase 진척 체크박스

- [x] **Phase A** — 프로젝트 스캐폴딩 (Vite+React+TS, COEP/COOP, hello_world WASM E2E) ← 2026-04-17 완료
- [x] **Phase A+** — g2o/(C)Sparse WASM 스파이크 ⚠️ 가장 중요 (Gate) ← 2026-04-18 통과
  - 결과: **✅ success** — 원안대로 Phase B 이후 진행. g2o 경로 채택 (Stage 1 PnP 통과 + Stage 2 BA via `solver_eigen`/`SimplicialLLT` 통과)
  - Open follow-up: CXSparse 자체의 Emscripten 빌드는 미검증 — Phase G에서 성능이 부족하면 그때 통합 재평가
- [x] **Phase B** — Step 1~2 Dataset + Camera ← 2026-04-25 완료
- [x] **Phase C** — Step 3~4 Feature Detection + Stereo LK ← 2026-04-26 완료
  - [x] 게이트 spike(Q1~Q4) ← 2026-04-26 통과 (OpenCV 4.13.0 분리 빌드 → static lib → find_package → cv::goodFeaturesToTrack + cv::calcOpticalFlowPyrLK 검증)
  - [x] Step 3 본 작업 — `bind_features.cpp`, AlgoPicker(GFTT/Harris/FAST/ORB), Step3 UI
  - [x] Step 4 본 작업 — LK 좌→우 매칭, Step4 UI
  - Phase C+로 deferred: SIMD/MT variant 빌드, WebGL Harris / WebGPU GFTT, OpenCV contrib(SIFT/AKAZE)
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
| 3. Feature Detection | ✅ | ✅ | `myslam_features.baseline` 1.14 MB. GFTT/Harris/FAST/ORB + 마스크. 게이트: WASM 로드, 좌측 이미지 로드, 검출수 ≥ max·0.5, 4×4 그리드 ≥ 8 셀 분포 |
| 4. Stereo Matching (LK) | ✅ | ✅ | LK pyramid (winSize/maxLevel/maxIter/eps/useInitialFlow). 게이트: 매칭율 ≥ 60%, mean &#124;dy&#124; ≤ 2 px, mean dx < 0 |
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
| 2026-04-26 | **Phase C 백엔드 경로: OpenCV 4.13 분리 빌드 + static lib 링크 (B′)** 채택 — `add_subdirectory(opencv)`(B), OpenCV.js 별도 런타임(A), 순수 C++ 자체 구현(C)을 모두 검토 후 선택 | (1) PLAN §1.3의 "OpenCV.js 4.10+ 커스텀 빌드"의 정신("ch13의 OpenCV 의존을 WASM에서 살리되 모듈 최소화")을 가장 충실히 구현. (2) 순수 add_subdirectory(B)는 OpenCV 4.13에서도 미지원 — issue [#27548](https://github.com/opencv/opencv/issues/27548) 2025-07 제기 후 9개월째 open. 4.x CMakeLists의 `${CMAKE_SOURCE_DIR}/modules/...` 절대경로 참조가 사용자 top-level project를 가리켜 깨짐. 비슷한 #26955도 open. (3) 그러나 BUILD_LIST + CV_DISABLE_OPTIMIZATION + WITH_*=OFF 단일 플래그로 모듈/SIMD/I-O 코덱 깔끔히 차단 가능 — 처음 우려했던 "per-arch SIMD 디스에이블 노가다"는 사실 단일 옵션으로 해결. (4) A안(OpenCV.js 별도)은 두 WASM HEAP 분리, .delete() 위생 부담, mt variant 워커 풀 분리 등 운영 복잡도 ↑ (sparse 점 데이터 cross-runtime 비용 자체는 ~2-3 ms/frame로 미미함을 확인). (5) C안(순수 자체 구현)은 학습 가치 높지만 FAST/ORB까지 작성 시 ch13 책 API와 수치적으로 어긋날 위험 + 향후 cv::solvePnPRansac 같은 Step 8 대체 알고리즘 도입 시 다시 OpenCV 재도입 필요. (6) **B′은 Phase A+ g2o 스파이크와 같은 정신**: 분리 빌드 + cmake에서 `find_package(OpenCV PATHS ... NO_DEFAULT_PATH)` — 이미 입증된 패턴 | Phase C 본 작업 전에 **spike(Q1~Q4) 통과 게이트** 필수. OpenCV submodule pin은 **tag 4.13.0**(2025-12-31, latest stable). 5.x는 alpha라 미채택. spike 결과에 따라 본 작업 ETA 1.5주~2주 예상. 향후 Step 8 PnP에서 calib3d 모듈을 BUILD_LIST에 추가하기만 하면 cv::solvePnPRansac 대체 알고리즘 가능 |
| 2026-04-26 | **OpenCV submodule shallow clone**: `git clone --depth 1 --branch 4.13.0 …`로 받고 `.gitmodules`에 `shallow = true` 명시 | OpenCV 풀 히스토리는 ~700 MB / 25 GB 잔여 디스크에서 부담. 단일 태그만 필요하므로 shallow 만으로 충분 | repo 추가 디스크 309 MB. 향후 4.13.x 패치로 올릴 때는 `git fetch --depth 1 origin tag <new>` 후 checkout |
| 2026-04-26 | **OpenCV CMake 통합 — `OpenCV_DIR` 직접 지정**: `find_package(OpenCV ... PATHS … NO_DEFAULT_PATH)`만으로는 `lib/cmake/opencv4/`의 `OpenCVConfig.cmake`를 매칭하지 못함 | 패키지 이름(`OpenCV`)과 디렉터리 이름(`opencv4`)이 case-insensitive로도 어긋남. CMake 3.20 기준 검증된 동작 | `set(OpenCV_DIR "${prefix}/lib/cmake/opencv4" CACHE PATH "" FORCE)` 후 `find_package(OpenCV 4.13 REQUIRED COMPONENTS … NO_DEFAULT_PATH)` 패턴으로 픽스. 향후 contrib 통합 시에도 동일 |
| 2026-04-26 | **`-fexceptions` + `DISABLE_EXCEPTION_CATCHING=0`을 `cv_spike` 타깃에 한정**(g2o spike 패턴 확장 X) | OpenCV는 `cv::Exception` throw, g2o spike도 마찬가지지만 `hello`/`camera`처럼 예외 없는 타깃은 zero-cost. PLAN §10 번들 가드 유지 | Step 3/4 본 바인딩(`bind_features.cpp`)에서 동일 플래그를 자기 타깃에만 추가 |
| 2026-04-26 | **TS 우선 규칙(memory도 갱신)**: 스파이크 검증 스크립트는 `wasm-src/spike/verify_cv.ts` + `node --experimental-strip-types` | `verify_pnp.mjs`/`verify_ba.mjs`(2026-04-18 작성)는 그대로 두되 신규 Node 스크립트는 모두 TS. `tsconfig.node.json` include에 `wasm-src/spike/**/*.ts` 추가 — `@types/node` 자동 적용 | 사용자 명시 피드백(2026-04-26): "please use typescript whenever possible". memory `feedback_typescript_default.md` 참조 |
| 2026-04-26 | **`bind_features` API: Float64Array 3-stride [x,y,score] 단일 표준** | cv_spike는 detect→2-stride, track 입력 또한 2-stride였으나 본 바인딩에서 stride 자동 감지가 모호함(N=3일 때 length=6은 2/3 둘 다 해석 가능). 단일 표준으로 통일하면 Step 3→4 파이프라인이 detect 출력을 그대로 trackLK에 넘길 수 있어 boilerplate 제거 | trackLK opts.initialPts도 동일 3-stride. score 필드는 LK에서 무시 |
| 2026-04-26 | **CMake 헬퍼 함수 `_ch13_import_opencv()` 도입** | cv_spike + features 두 타깃이 동일한 `find_package(OpenCV …)` 절차를 반복 — DRY 위반 + 향후 OpenCV consumer 추가 시 cargo-cult 위험 | `wasm-src/CMakeLists.txt` 안에서만 사용. 다른 옵션(빌드 트리 변경, contrib 추가)도 단일 진입점에서 관리 |
| _(미정)_ | OpenCV submodule 채택 시 contrib(SIFT/AKAZE)까지 포함할지 여부 | features2d만으로 GFTT/Harris/FAST/ORB 충족 — contrib는 Step 11 BA·Loop closure 시점에 재평가 | 번들 크기 |
| _(미정)_ | Phase C+ — SIMD/MT variant 빌드 도입 시점 | 현재 baseline만 빌드. PLAN §4 매트릭스의 Step 3/4 행 활성화는 Phase G(BA) spike 결과로 성능 병목 위치를 확인한 뒤 일괄 도입이 효율적 | OpenCV 분리 빌드를 variant마다 1회씩 추가로 돌려야 함 (~5분 × 4) |

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
  - WASM Features 바인딩 (Step 3+4): `ch13-wasm/wasm-src/myslam/bindings/bind_features.cpp`
  - Phase A+ 스파이크: `ch13-wasm/wasm-src/spike/{bind_pnp_spike.cpp, bind_ba_spike.cpp, verify_pnp.mjs, verify_ba.mjs, verify_pnp_diag*.mjs}`
  - Phase C 게이트 스파이크: `ch13-wasm/wasm-src/spike/{bind_cv_spike.cpp, verify_cv.ts}` — `npm run verify:cv_spike`
  - Phase C 본 작업 검증: `ch13-wasm/wasm-src/spike/verify_features.ts` — `npm run verify:features`
  - 빌드 스크립트: `ch13-wasm/wasm-src/{CMakeLists.txt, build.sh}` — `bash build.sh [baseline|simd|mt|mt-simd] [hello|camera|pnp_spike|ba_spike|cv_spike|features]`
  - OpenCV 분리 빌드 스크립트: `ch13-wasm/wasm-src/scripts/build-opencv.sh` — `npm run build:opencv:baseline`
  - g2o submodule (modern master): `ch13-wasm/wasm-src/third_party/g2o/`
  - Eigen submodule (tag 5.0.1): `ch13-wasm/wasm-src/third_party/eigen/`
  - OpenCV submodule (tag 4.13.0, shallow): `ch13-wasm/wasm-src/third_party/opencv/`
  - OpenCV 빌드 산출물: `ch13-wasm/wasm-src/build/opencv-{build,install}-<variant>/` (gitignored). install/lib에 `libopencv_{core,imgproc,features2d,video}.a`
  - 빌드 산출물: `ch13-wasm/public/wasm/myslam_{hello,camera,pnp_spike,ba_spike,cv_spike,features}.<variant>.{js,wasm}`
  - Step 3 UI: `ch13-wasm/src/steps/Step03_FeatureDetection/`
  - Step 4 UI: `ch13-wasm/src/steps/Step04_StereoMatching/`
  - TS 로더: `ch13-wasm/src/wasm/features.ts` (`loadFeaturesWasm` + `imageDataToGray`)
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

- [x] `http://localhost:5173/step/dataset`: 좌/우 체커보드 이미지에 스테레오 disparity(우측 이미지가 좌측보다 좌로 ~22 px 이동)가 보이는지 확인
- [x] `http://localhost:5173/step/camera`: 슬라이더를 움직여도 "60-point grid max error" 표시값이 1e-13 수준에 머무는지 확인, P0→P1 전환 시 extrinsic t가 [0,0,0] → [-0.5372,0,0]으로 바뀌는지 확인

---

## Phase C — 게이트 spike 결과 (2026-04-26)

### 결과: ✅ 4개 게이트 모두 통과 → 본 작업 착수 승인

`npm run build:opencv:baseline && npm run build:wasm:cv_spike && npm run verify:cv_spike` 한 줄 흐름으로 재현 가능.

| Q | 질문 | 결과 |
|---|------|------|
| Q1 | OpenCV 4.13.0 submodule이 emcc + `BUILD_LIST=core,imgproc,features2d,video` + disable 플래그 조합으로 빌드되는가? | ✅ Configure 135.7s + Build ~5분. `wasm-src/scripts/build-opencv.sh baseline` 단일 실행 |
| Q2 | static `.a` + 헤더가 install prefix에 떨어지는가? | ✅ `lib/libopencv_{core,imgproc,features2d,video}.a` (총 8.1 MB) + `include/opencv4/opencv2/` + `lib/cmake/opencv4/OpenCVConfig.cmake` |
| Q3 | top-level CMakeLists에서 `find_package(OpenCV)`로 detect되고 `cv_spike` 타깃에 링크되는가? | ✅ `OpenCV_DIR`을 `lib/cmake/opencv4`에 직접 set 후 `find_package(... NO_DEFAULT_PATH)`로 픽스. 단일 `myslam_cv_spike.baseline.wasm` 750 KB(g2o 스파이크 대비 +약 2배 — 합리적) |
| Q4 | KITTI mini frame 0/1에서 GFTT + LK가 합리적 결과를 내는가? | ✅ GFTT 200 corners(요청 200, 100% saturate), LK 200/200 tracked, **mean dx = −14.00 px / dy = −0.00 px** (예상 forwardShift = −14 px, dy = 0 정확 일치) |

### 본 작업 착수 시 재사용할 패턴

1. `bind_features.cpp`는 `bind_cv_spike.cpp`의 detectGFTT/trackLK API를 확장 — 알고리즘 enum(GFTT/Harris/FAST/ORB), 옵션 객체, 마스크 인자 추가.
2. CMake 통합: cv_spike 타깃이 사용한 `set(OpenCV_DIR …)` + `find_package(OpenCV 4.13 REQUIRED COMPONENTS … NO_DEFAULT_PATH)` + `target_link_libraries(... ${OpenCV_LIBS})` + `-fexceptions/-sDISABLE_EXCEPTION_CATCHING=0`을 그대로 새 타깃에 복제.
3. SIMD/MT variant 빌드는 `build-opencv.sh simd` 등을 추가 호출해 `opencv-install-simd/`를 만들고 `CH13_WASM_VARIANT=simd`로 상위 빌드. **현재 spike에서는 baseline만 검증** — Phase C+ 또는 Phase G에서 일괄 도입.

### 디스크 footprint (참고)

| 디렉토리 | 크기 |
|----------|------|
| `wasm-src/third_party/opencv/` (shallow 4.13.0) | 309 MB |
| `wasm-src/build/opencv-build-baseline/` (gitignored) | 37 MB |
| `wasm-src/build/opencv-install-baseline/` (gitignored) | 22 MB |
| `wasm-src/build/cv_spike_baseline/` (gitignored) | 1.1 MB |
| `public/wasm/myslam_cv_spike.baseline.{js,wasm}` | 87 KB + 750 KB |

---

## Phase C — 착수 계획 (2026-04-26)

### 채택 경로: **B′ — OpenCV 4.13 분리 빌드 + static lib 링크**

본 Phase는 PLAN.md §9 Phase C("OpenCV.js minimal 빌드 + GFTT/LK + Harris/FAST/ORB + WebGL/WebGPU + MT/SIMD")의 1.5주 규모 작업이라 한 세션에 모두 다룰 수 없다. 백엔드 경로 결정과 Phase A+ 스타일 spike를 먼저 분리해 진행한다.

### 검토한 4가지 경로

| 안 | 요약 | 채택? |
|----|------|------|
| A | OpenCV.js 별도 WASM 런타임 + JS 코디네이터 | △ (가능, 비용 ~2-3 ms/frame, 운영 복잡 ↑) |
| B | g2o처럼 `add_subdirectory(opencv)` 시도 | ❌ **막힘** — issue [#27548](https://github.com/opencv/opencv/issues/27548) 미해결 (2025-07~), 4.x CMake가 `${CMAKE_SOURCE_DIR}/modules/...` 절대경로로 사용자 top-level project를 가리킴 |
| **B′** | **OpenCV 별도 빌드 → static lib(.a) → 우리 myslam 타깃에 링크** | ✅ **채택** |
| C | 순수 C++ 자체 GFTT/Harris/LK 작성 | △ (학습 가치 ↑, 향후 calib3d 재도입 시 부담 ↑) |

### B′ 선택 근거 (요약)

1. **PLAN.md §1.3 "OpenCV.js 4.10+ 커스텀 빌드"의 정신**(ch13 OpenCV 의존을 살리되 모듈 최소)을 가장 충실히 구현.
2. 4.x `CMakeLists.txt` 분석 결과 (April 2026 기준):
   - `BUILD_LIST=core,imgproc,features2d,video` (line 285) — 모듈 cherry-pick 일급 지원.
   - `CV_DISABLE_OPTIMIZATION=ON` (line 1050) — SIMD intrinsic 단일 플래그로 차단.
   - `WITH_JPEG/PNG/TIFF/WEBP/PROTOBUF/IPP=OFF` — image I/O 코덱 깔끔히 제거(이미지는 JS `createImageBitmap`으로 디코드).
   - `if(EMSCRIPTEN)` 분기 (line 1486) — 시스템 라이브러리 자동 스킵.
3. **단일 myslam.wasm 안에서 g2o + OpenCV가 함께 링크** → 두 런타임 cross-runtime overhead 0, `.delete()` 위생 부담 없음.
4. 향후 Step 8 PnP에서 cv::solvePnPRansac을 추가할 때 `BUILD_LIST`에 `calib3d` 추가만 하면 됨.
5. 이미 입증된 패턴: Phase A+에서 g2o submodule + cmake `find_package(g2o ...)`로 동작 확인.

### 게이트 — Phase C 본 작업 착수 전 필수 spike (Q1~Q4)

본 작업(`bind_features.cpp`, Step 3/4 UI, AlgoPicker)에 들어가기 전에 다음 4개 질문에 답한다. Phase A+ g2o 스파이크와 같은 정신.

- **Q1**: OpenCV 4.13.0 submodule이 emscripten toolchain + `BUILD_LIST=core,imgproc,features2d,video` + 위 disable 플래그 조합으로 빌드 완료되는가? (예상 30~60분)
- **Q2**: `BUILD_SHARED_LIBS=OFF` + `BUILD_opencv_js=OFF` 로 install_prefix에 `libopencv_core.a`, `libopencv_imgproc.a`, `libopencv_features2d.a`, `libopencv_video.a` + 헤더가 떨어지는가?
- **Q3**: 우리 `wasm-src/CMakeLists.txt`에서 `find_package(OpenCV 4.13 REQUIRED PATHS .../opencv-install NO_DEFAULT_PATH)` 로 detect되고, 새 `cv_spike` 타깃에 `OpenCV::core`/`imgproc`/`features2d`/`video` 가 링크되는가?
- **Q4**: 가장 단순한 `cv::goodFeaturesToTrack` + `cv::calcOpticalFlowPyrLK` 호출이 KITTI mini frame 0에서 합리적 결과(특징점 수 ≈ 150, 분포가 체커보드 코너 근처)를 내는가? `verify_cv.mjs` Node 스모크.

**Gate 판정**:
- ✅ 4개 모두 통과 → 본 작업 착수.
- ❌ Q1/Q2 실패 (빌드 자체 막힘) → A안(OpenCV.js 별도 런타임)으로 fallback.
- ❌ Q3 실패 (링크 단계) → 빌드는 되지만 통합 미해결 — issue 분석 후 ExternalProject_Add 또는 직접 `target_link_libraries(... ${OpenCV_LIBS})` 시도.
- ❌ Q4 실패 (수치 이상) → spike 환경 자체 디버깅(이미지 디코딩 / Mat 변환).

### 본 작업 (게이트 통과 후)

PLAN §9 Phase C 1.5주 추정과 일치. 단, OpenCV.js 빌드 대신 **분리 빌드 + 링크**로 구현.

1. `bind_features.cpp` — `detectFeatures(rgba, w, h, opts)` (algo: GFTT/Harris/FAST/ORB), `trackLK(prev, curr, w, h, pts, opts)`. Embind 인터페이스는 기존 `bind_camera.cpp` 패턴과 동일.
2. CMake 통합 — `wasm-src/third_party/opencv/`(submodule) + 별도 빌드 디렉토리 + `find_package`. `build.sh`에 `features` 타깃 추가 + 첫 빌드 캐시 안내.
3. TS 로더 `src/wasm/features.ts` (기존 `camera.ts` 패턴).
4. Step 3 UI: AlgoPicker(GFTT/Harris/FAST/ORB), maxFeatures/minDist/qualityLevel/blockSize 슬라이더, 좌 이미지 위 keypoint 오버레이, VerifyGate(검출 수 ≈ 설정값, 분포 균등성).
5. Step 4 UI: Step 3 결과 → `cv::calcOpticalFlowPyrLK` → 좌→우 대응선 시각화, VerifyGate(매칭율 ≥ 60%).
6. SIMD/MT variant, WebGL/WebGPU 가속 경로는 **Phase C+로 분리** (PLAN §4 매트릭스의 Step 3/4 행).

### 의식적으로 deferred

- FAST/ORB 외 SIFT/AKAZE 등 contrib 모듈 — features2d로 충족, contrib 통합은 Phase 후반에 재평가.
- WebGL Harris / WebGPU GFTT — Phase C에서는 OpenCV 경로만. Phase C+ 또는 Phase I 마감 단계에서 추가.
- WASM SIMD/MT variant — Phase G(BA) spike에서 성능 측정 후 일괄 도입 예정.

---

## Phase C — Step 3/4 본 작업 완료 (2026-04-26)

### 결과 요약

| 항목 | 값 |
|------|-----|
| `bind_features.cpp` | 247 LOC. Embind: `detectFeatures(grayU8, w, h, algo, opts)` + `trackLK(prev, curr, w, h, prevPts, opts)` + `opencvVersion()` + 4개 `DETECTOR_*` enum |
| 지원 알고리즘 | GFTT / Harris (`cv::goodFeaturesToTrack(useHarris=true)`) / FAST (`cv::FAST` + 마스크 후처리 + nth_element 컷오프) / ORB (`cv::ORB::create`) |
| LK API | `winSize`, `maxLevel`, `maxIter`, `eps`, `useInitialFlow`, `initialPts` 모두 노출. ch13의 `OPTFLOW_USE_INITIAL_FLOW` 패턴 그대로 |
| WASM 크기 | `myslam_features.baseline.{js, wasm}` = 88 KB + **1143 KB** (cv_spike 750 KB 대비 +52% — ORB/FAST 코드 추가) |
| 빌드 시간 | 단일 .cpp → ~6초 (OpenCV는 캐시 재사용) |

### `verify_features.ts` 검증 결과 (`npm run verify:features`)

| 케이스 | 결과 |
|--------|------|
| OpenCV version | 4.13.0 ✅ |
| GFTT detect (synthetic frame 0, maxFeatures=200) | 200 / 200 in 211 ms |
| Harris detect | 200 / 200 in 213 ms |
| FAST detect | 200 / 200 in 210 ms |
| ORB detect | 202 in 211 ms |
| GFTT + 100×100 mask hole | 200 kps, **0** inside hole (mask 정상) |
| LK 좌→우(no init), expected dx ≈ −22 | tracked 200/200 (100%), **mean dx = −21.95**, mean dy = −0.00 |
| LK with `useInitialFlow` + GT-shifted seeds | tracked 200/200, mean dx = −21.95 (동일 수치, init 정상) |

### Step 3/4 UI 헤드리스 검증 (2026-04-26)

| 항목 | 관측값 |
|------|--------|
| `GET /step/feature-detection` | 200 text/html (SPA fallback) |
| `GET /step/stereo-matching`   | 200 text/html (SPA fallback) |
| `GET /wasm/myslam_features.baseline.js` | 200 text/javascript, 90 083 B, COEP/COOP 반영 |
| `GET /wasm/myslam_features.baseline.wasm` | 200 application/wasm, 1 143 809 B, COEP/COOP 반영 |
| `npm run build` | tsc -b clean + vite build OK — `dist/assets/index-*.js` = 287.84 KB (gzip **92.25 KB**, PLAN §10 ≤ 400 KB 충족) |

### Step 3 — Feature Detection (UI)

- 4개 알고리즘 토글 (GFTT/Harris/FAST/ORB). 알고리즘별 적용 가능한 슬라이더만 표시 (예: `qualityLevel`/`minDistance`/`blockSize`는 GFTT/Harris, `fastThreshold`는 FAST, `orbScaleFactor`/`orbNLevels`는 ORB).
- frame slider (0..4), `maxFeatures` slider (20..500).
- 좌측 입력: KITTI mini fixture left 이미지를 `imageDataToGray`로 8-bit grayscale 변환.
- 우측 출력: 동일 이미지 위에 검출된 keypoint를 녹색 원으로 오버레이 + 검출 시간(ms) + 4×4 그리드 coverage(N/16).
- VerifyGate 자동 게이트:
  1. WASM 모듈 로드 (OpenCV 4.13.0 매칭)
  2. 좌측 이미지 로드
  3. 검출 수 ≥ max(30, maxFeatures × 0.5)
  4. 4×4 그리드 ≥ 8 cells에 분포 (한쪽 쏠림 가드)

### Step 4 — Stereo Matching (LK)

- Step 3와 같은 detector 토글 + LK pyramid 슬라이더 (winSize 5..31, maxLevel 0..5, maxIter 5..60, eps 0.001..0.1) + `useInitialFlow` 체크박스.
- 좌측 입력: stereo pair 좌/우를 vertical stack으로 표시 + baseline(m) 표시.
- 우측 출력: 좌(상단) + 우(하단) 합성 캔버스에 매칭 라인 오버레이. 녹색=추적 성공, 빨강=실패.
- 요약 표시: `tracked`, `mean dx`, `mean |dy|`, detect ms, LK ms.
- VerifyGate 자동 게이트:
  1. WASM 모듈 로드
  2. 좌/우 프레임 + calib 로드
  3. 매칭율 ≥ 60% (PLAN §3 Step 4 검증 기준)
  4. 평균 epipolar `|dy|` ≤ 2 px (PLAN §3 Step 4 검증 기준)
  5. 평균 disparity dx < 0 (우측이 좌로 이동했음을 sign으로 확인)

### 의식적으로 deferred (Phase C+로 분리)

- WASM SIMD/MT variant 빌드 — Phase G BA spike 후 일괄 도입.
- WebGL Harris / WebGPU GFTT — Phase I 마감 단계에서 합류 검토.
- 알고리즘 교차 벤치마크 표 (`benchStore` 통합) — `PerfMeter`가 stub인 채라 PerfMeter 본 구현 시점에 추가.
- 학습 노트 `docs/steps/03-feature-detection.md`, `04-stereo-matching.md` — Phase I 문서화 단계.

### 남은 사용자 육안 관측

- `http://localhost:5173/step/feature-detection`: 알고리즘 토글 시 keypoint 오버레이가 4개 패턴(GFTT는 코너 집중, FAST는 임계값에 따른 샤프함, ORB는 스케일 분포)으로 변하는지 확인.
- `http://localhost:5173/step/stereo-matching`: 매칭 라인이 거의 수평하고 (epipolar) 좌→우 방향으로 수십 px 이동하는지 확인. `useInitialFlow` 토글 후 평균 dx가 더 stable해지는지 비교.

---

## 갱신 규칙

1. **Phase/Step 착수 시**: "현재 진행 중" 필드를 갱신.
2. **완료 시**: 해당 체크박스 ✅, "검증 게이트 통과"에 통과 조건 요약을 기록.
3. **결정이 발생하면**: "주요 결정 로그"에 append-only로 한 줄 추가.
4. **블로커 발생**: "현재 블로커"에 원인·시도한 해결책·다음 행동을 기록.
5. **세션 종료 시**: 반드시 이 파일 갱신 후 커밋. 커밋 메시지 예: `progress: finish Phase A scaffolding`.
