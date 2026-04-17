---
title: ch13 myslam WASM 교육용 플레이그라운드 PLAN
based_on: docs/analysis/2026-04-17-ch13-full-analysis.md
target: slambook2/ch13 Stereo Visual Odometry → WebAssembly + Vite/React/TS
created: 2026-04-17
---

# ch13 myslam WASM 교육용 플레이그라운드 구현 계획서

이 문서는 slambook2 ch13(Stereo Visual Odometry)의 C++ 구현을 브라우저에서 단계별로 학습·실험할 수 있는 **WASM 기반 교육용 플레이그라운드**를 만들기 위한 상세 계획(prompt)입니다.  
본 문서만 주어져도 구현을 시작할 수 있도록 **아키텍처·빌드 체인·단계별 산출물·검증 기준**까지 구체적으로 기술합니다.

---

## 0. 목표 요약

1. ch13의 C++ 파이프라인(Dataset → Frontend → Backend → Map → Viewer)을 **WASM으로 포팅**하여 브라우저에서 실행한다.
2. **Vite + React + TypeScript** 프론트엔드에서 각 단계를 **시각적 교육 화면**으로 노출한다.
3. 학습자가 **파라미터와 알고리즘을 실시간으로 교체/조합**하며 결과를 비교할 수 있다.
4. 적용 가능한 단계에서는 **Multi-Thread (pthreads + SharedArrayBuffer)**, **WASM SIMD**, **WebGL**, **WebGPU** 가속을 선택·비교할 수 있다.
5. 각 단계는 **"검증 체크리스트"를 통과해야 다음 단계로 진행**할 수 있는 게이트 구조를 갖는다.

### 0.1 본 문서를 프롬프트 파일로 사용하는 법

이 문서는 구현을 주도할 LLM 에이전트(또는 팀)에게 전달할 **입력 명세**로 설계되었다.

- §9의 각 Phase는 **독립 세션으로 실행 가능**하도록 자립적으로 기술되어 있다. 한 세션에서 한 Phase를 끝내고 PR을 만든 뒤 다음 Phase를 새 세션으로 시작해도 무방하다.
- §3의 단계 카드 스키마와 부록 A의 체크리스트는 **그대로 복사해 각 단계 구현 프롬프트에 붙여 사용**한다.
- 구현 중 계획과 현실이 어긋나면 본 문서를 **정적 스펙이 아닌 살아있는 계약**으로 취급해 수정 PR과 함께 갱신한다.
- 본 문서에 없는 결정은 §13의 참고 자료와 `docs/analysis/2026-04-17-ch13-full-analysis.md`를 1차 출처로 삼는다.

#### 0.1.1 세션 재시작 프로토콜

컨텍스트를 비우고 새 세션에서 이어서 진행할 때는 **반드시 아래 순서**를 따른다.

1. **먼저 읽기 (필수)**
   - `PLAN.md` (본 문서)
   - `PROGRESS.md` (진행 상태·결정 기록 — 각 Phase 종료 시 갱신됨)
   - `docs/analysis/2026-04-17-ch13-full-analysis.md` (원본 코드 분석)
2. **현재 위치 파악**: `PROGRESS.md`의 "현재 진행 중인 Phase" 항목과 마지막 완료 체크박스를 확인.
3. **작업 실행**: 다음 미완료 Phase/Step의 §3 또는 §9 해당 섹션을 그대로 수행.
4. **종료 시**: `PROGRESS.md`를 갱신 + 커밋(체크박스·결정·블로커 반영) → 그래야 **다음 세션이 상태를 복원**할 수 있다.

**권장 세션 시작 프롬프트(복사해 사용)**:

```
PLAN.md와 PROGRESS.md와 docs/analysis/2026-04-17-ch13-full-analysis.md를 먼저 읽어 줘.
PROGRESS.md의 "현재 진행 중" 항목을 확인하고, 이어서 [Phase X / Step N]를 진행해 줘.
완료 후에는 PROGRESS.md를 갱신하고 커밋해 줘.
```

(특정 Step만 실행할 때: `[Phase X / Step N]`을 해당 값으로 교체. 상태 확인만 원할 때는 마지막 문장을 "현재 상태를 요약해 줘"로 교체.)

---

## 1. 기술 스택

### 1.1 프론트엔드

| 항목 | 선정 | 이유 |
|------|------|------|
| 번들러 | Vite 5+ | COEP/COOP 헤더 쉬운 설정, WASM 핫리로드 |
| 언어 | TypeScript 5+ | 타입 안전한 WASM 바인딩 |
| UI | React 18 | 단계별 화면 구성에 적합 |
| 상태관리 | Zustand | 파이프라인 상태/파라미터 스토어 |
| 2D 렌더링 | Canvas 2D + WebGL(regl) | 이미지·특징점·플로우 오버레이 |
| 3D 렌더링 | Three.js (r160+) | 카메라 궤적·포인트 클라우드 |
| WebGPU | 네이티브 WebGPU API | 가속 선택 시 |
| 차트 | uPlot | 성능 측정 시계열 |
| 코드 하이라이트 | Shiki | C++ 코드 학습용 표시 |

### 1.2 WASM 빌드 체인

| 항목 | 선정 | 비고 |
|------|------|------|
| 컴파일러 | Emscripten 3.1.61+ | `-pthread`, `-msimd128`, `-sUSE_WEBGPU` 지원 |
| 빌드 시스템 | CMake + Emscripten toolchain | 기존 CMakeLists.txt 재활용 |
| 패키지 관리 | emsdk vendoring + submodule | 재현 가능성 |

### 1.3 WASM 대상 C/C++ 의존성 이식 전략

| 원 의존성 | 이식 방법 | 비고 |
|-----------|-----------|------|
| Eigen3 | header-only 그대로 | SIMD 타겟 시 `-msimd128 -DEIGEN_VECTORIZE_SSE` |
| Sophus | header-only 그대로 | C++17 호환성 확인 |
| OpenCV | **OpenCV.js 4.10+ 커스텀 빌드** | GFTT, LK, ORB만 포함하는 minimal build |
| g2o | **소스 포함 후 Emscripten 빌드** | CSparse 대신 **SuiteSparse-CXSparse** 또는 Dense 솔버 |
| CSparse | CXSparse 소스 직접 포함 | MIT 라이선스 |
| glog | **로거 스텁** (console.log로 대체) | 빌드 부담 제거 |
| gflags | **제거** — 파라미터는 JS에서 전달 | Embind로 `Config` 구조체 전달 |
| Pangolin | **제거** — Three.js + WebGL/WebGPU로 대체 | Viewer는 TS로 완전 재작성 |
| GTest | **제거** — Vitest로 TS 측 테스트 | 필요 시 Catch2 WASM |
| pthread | Emscripten pthreads | COEP/COOP 헤더 필수 |

> 원본 ch13은 `-fopenmp`를 사용하지만 Emscripten의 OpenMP 지원은 한정적이므로 본 프로젝트에서는 **OpenMP 대신 Emscripten pthreads를 직접 사용**한다. 원본 코드의 `#pragma omp` 영역은 수동 worker 분산 로직으로 대체한다.

> 원본 ch13 CMakeLists는 `-std=c++11`로 고정되어 있으나, Sophus 최신 버전은 C++17을 요구할 수 있다. 빌드 시 **Sophus 버전을 ch13과 호환되는 릴리스에 고정**하거나, 전체 코드를 C++17로 올려 빌드한다. 결정은 Phase A+ 스파이크에서 확정.

### 1.4 브라우저 요구사항

- Chrome/Edge 121+ (WebGPU 기본 활성화), Safari 17.4+ (WebGPU 플래그)
- `Cross-Origin-Opener-Policy: same-origin`
- `Cross-Origin-Embedder-Policy: require-corp`
- SharedArrayBuffer 활성화 필요(멀티스레드 경로)

---

## 2. 디렉토리 구조

```
ch13-wasm/                               ← 본 프로젝트 루트 (신규)
├── README.md                            ← 빠른 시작·스크린샷 (구현 시 작성)
├── index.html
├── package.json
├── vite.config.ts                       ← COEP/COOP 헤더 설정
├── tsconfig.json
├── public/
│   ├── datasets/                        ← 경량 KITTI 샘플 (5~10 프레임)
│   │   └── kitti05-mini/
│   │       ├── image_0/  image_1/  calib.txt
│   └── wasm/                            ← 빌드된 .wasm, .js 글루
├── wasm-src/                            ← C/C++ 소스 & 빌드 스크립트
│   ├── CMakeLists.txt                   ← ch13 루트 CMake 재사용 + Emscripten toolchain
│   ├── emscripten_toolchain.cmake
│   ├── build.sh                         ← 빌드 진입점 (variants 지정)
│   ├── variants/                        ← 가속 조합별 산출물
│   │   ├── baseline/                    ← 순수 scalar
│   │   ├── simd/                        ← -msimd128
│   │   ├── threads/                     ← -pthread
│   │   └── simd-threads/                ← 둘 다
│   ├── myslam/                          ← ch13/src, ch13/include 복사 + 수정
│   │   ├── include/   src/
│   │   └── bindings/                    ← Embind 바인딩 (step별 진입점)
│   │       ├── bind_dataset.cpp
│   │       ├── bind_features.cpp
│   │       ├── bind_triangulation.cpp
│   │       ├── bind_pnp.cpp
│   │       ├── bind_ba.cpp
│   │       └── bind_vo.cpp              ← 전체 파이프라인
│   └── third_party/
│       ├── eigen/       sophus/
│       ├── opencv-minimal/              ← 전용 빌드 프로파일
│       ├── g2o/         cxsparse/
├── src/                                 ← React/TS 프론트엔드
│   ├── main.tsx   App.tsx   index.css
│   ├── wasm/
│   │   ├── loader.ts                    ← WASM 변형 로더 (baseline/simd/threads)
│   │   ├── types.ts                     ← Embind로 노출된 타입 선언
│   │   └── workers/                     ← 각 스텝 Web Worker
│   │       ├── features.worker.ts
│   │       ├── lk.worker.ts
│   │       ├── ba.worker.ts
│   │       └── vo.worker.ts
│   ├── gpu/
│   │   ├── webgl/                       ← regl 기반 2D 셰이더 커널
│   │   │   ├── gftt.ts   lk.ts   overlay.ts
│   │   └── webgpu/                      ← compute shader
│   │       ├── gftt.wgsl   lk.wgsl   ba_residuals.wgsl
│   ├── steps/                           ← 단계별 화면 컴포넌트
│   │   ├── Step01_Dataset/
│   │   ├── Step02_Camera/
│   │   ├── Step03_FeatureDetection/
│   │   ├── Step04_StereoMatching/
│   │   ├── Step05_Triangulation/
│   │   ├── Step06_InitialMap/
│   │   ├── Step07_FrameTracking/
│   │   ├── Step08_PoseEstimation/
│   │   ├── Step09_Keyframe/
│   │   ├── Step10_NewMapPoints/
│   │   ├── Step11_BundleAdjustment/
│   │   ├── Step12_SlidingWindow/
│   │   └── Step13_FullPipeline/
│   ├── components/
│   │   ├── ParamPanel.tsx               ← 슬라이더·드롭다운 자동 생성
│   │   ├── AlgoPicker.tsx               ← 알고리즘 교체 UI
│   │   ├── AccelSwitch.tsx              ← MT/SIMD/WebGL/WebGPU 토글
│   │   ├── PerfMeter.tsx                ← ms·FPS·메모리
│   │   ├── ImageView.tsx                ← 이미지 + 오버레이
│   │   ├── Scene3D.tsx                  ← Three.js 궤적·포인트클라우드
│   │   ├── CodePeek.tsx                 ← 원본 C++ 코드 링크·하이라이트
│   │   ├── VerifyGate.tsx               ← 다음 단계 진행 게이트
│   │   └── StepLayout.tsx               ← 단계 공통 레이아웃
│   ├── state/
│   │   ├── pipelineStore.ts             ← 진행 상태, 검증 플래그
│   │   ├── paramStore.ts                ← 단계별 파라미터
│   │   └── benchStore.ts                ← 벤치마크 결과 시계열
│   └── lib/
│       ├── kitti.ts                     ← KITTI 파서 (브라우저)
│       ├── benchmark.ts                 ← perf.now 기반 측정 유틸
│       └── diff.ts                      ← 알고리즘 간 결과 비교
└── docs/
    ├── analysis/2026-04-17-ch13-full-analysis.md   ← 이미 있음 (참조)
    └── steps/                           ← 각 단계 학습 노트 (MD)
        ├── 01-dataset.md   02-camera.md   ...   13-full-pipeline.md
```

---

## 3. 단계별 커리큘럼 (13 steps)

각 단계는 다음 공통 스키마를 따릅니다.

> **단계 카드 스키마**  
> - **학습 목표**: 왜 이 단계가 필요한가  
> - **C++ 원본**: 대응하는 ch13 파일/함수  
> - **입력**: 타입·의미·시각화  
> - **출력**: 타입·의미·시각화  
> - **파라미터**: UI 노출 대상  
> - **교체 가능 알고리즘**: 드롭다운 선택지  
> - **가속 적용성**: MT/SIMD/WebGL/WebGPU 체크표  
> - **검증 체크리스트**: 통과해야 다음 단계 해금  
> - **의도된 실패 케이스**: 학습용 버그 주입 토글

### Step 1 — Dataset Loader (KITTI)

- **학습 목표**: 스테레오 쌍과 캘리브레이션이 어떻게 구성되는가
- **C++ 원본**: `src/dataset.cpp`, `include/myslam/dataset.h`
- **입력**: `public/datasets/kitti05-mini/` (10프레임 경량 샘플, gzip된 PNG)
- **출력**: `Frame` (left/right `cv::Mat` → JS에서는 `ImageData`), `Camera×4` (P0~P3 파싱)
- **파라미터**: 다운샘플 비율 (0.25 / 0.5 / 1.0), 시작 프레임 인덱스
- **교체 가능 알고리즘**:
  - 이미지 디코딩: 브라우저 네이티브 `ImageBitmap` vs WASM 측 `stb_image`
- **가속 적용성**: 없음(I/O 위주), 멀티워커 프리페치만
- **검증**: left/right 해상도·캘리브레이션 행렬·baseline 값 표시 및 수동 확인 체크박스
- **의도된 실패**: calib.txt 손상 파일 토글 → 에러 경로 관찰

### Step 2 — Camera Model

- **학습 목표**: 핀홀 투영/역투영과 스테레오 baseline
- **C++ 원본**: `include/myslam/camera.h`, `src/camera.cpp`
- **입력**: Step1의 Camera + 임의의 3D 점 (마우스로 드래그)
- **출력**: 2D 투영 좌표(left/right), 재투영 역방향
- **파라미터**: fx, fy, cx, cy 슬라이더 (실시간 왜곡 관찰)
- **교체 가능 알고리즘**: 없음 (모델 자체 학습 목적)
- **가속 적용성**: 없음
- **검증**: 재투영 오차 < 1e-5 (자체 검증)

### Step 3 — Feature Detection

- **학습 목표**: "좋은 특징점"의 정의와 검출기의 차이
- **C++ 원본**: `Frontend::DetectFeatures` (`src/frontend.cpp`)
- **입력**: left 이미지(회색조)
- **출력**: 2D 점 배열 + 각 점의 score
- **파라미터**: `num_features` (10–500), min distance, quality level, 마스크 반경
- **교체 가능 알고리즘**:
  - `GFTT` (원본)
  - `FAST` (OpenCV)
  - `ORB` keypoints
  - `Harris` (직접 구현 교육용)
- **가속 적용성**: ✅ MT (타일 분할), ✅ SIMD (고유값 계산), ✅ WebGL (fragment shader Harris), ✅ WebGPU (compute)
- **검증**: 결과 점 수가 설정값 근처, 시각적으로 균등 분포 확인 게이트
- **벤치**: 4가지 가속 조합 × 4가지 알고리즘 매트릭스 → uPlot 표시
- **의도된 실패**: 검출 반경을 0으로 설정 → 점 집중 현상 관찰

### Step 4 — Stereo Matching (LK on right image)

- **학습 목표**: 광학 흐름 vs 특징 기술자 매칭의 차이
- **C++ 원본**: `Frontend::FindFeaturesInRight`
- **입력**: left 이미지 + left 특징점, right 이미지
- **출력**: right 이미지에서의 대응점, 성공/실패 마스크
- **파라미터**: 피라미드 레벨(1–5), 윈도우 크기(7–31), 최대 반복(10–50), eps
- **교체 가능 알고리즘**:
  - LK Pyramid (원본)
  - ORB descriptor + BF Hamming
  - SGM(disparity 전처리 후 매칭)
  - Epipolar 1D search (learning용 brute-force)
- **가속 적용성**: ✅ MT (점 단위 분할), ✅ SIMD (gradient 윈도우), ✅ WebGL, ✅ WebGPU
- **검증**: 매칭율 > 60%, epipolar 오차 평균 < 2px
- **시각화**: left→right로 뻗어가는 대응선, 실패 점은 빨강

### Step 5 — Triangulation (Linear SVD)

- **학습 목표**: DLT 삼각화와 해 품질 판정
- **C++ 원본**: `include/myslam/algorithm.h::triangulation`
- **입력**: Step4의 left/right 대응점 쌍, 두 카메라의 투영 행렬
- **출력**: 3D 점 배열, 각 점의 σ4/σ3 조건수
- **파라미터**: 품질 임계값(1e-3 – 1e-1), Tcw 선택(identity/현재)
- **교체 가능 알고리즘**:
  - Linear SVD (원본)
  - Midpoint
  - Optimal Triangulation (Hartley-Sturm)
- **가속 적용성**: ✅ MT (점 단위), ✅ SIMD (4×4 SVD 벡터화), ✅ WebGPU (compute)
- **검증**: depth > 0 비율 확인, 재투영 오차 ≤ 1px 비율 표시
- **학습 포인트**: 분석 문서의 **반환 로직 버그** 재현 토글(`inverted-return`) — 학습자에게 원서 버그를 직접 경험시키기

### Step 6 — Initial Map Construction

- **학습 목표**: 첫 키프레임 + 초기 지도 구성
- **C++ 원본**: `Frontend::BuildInitMap`
- **입력**: Step3–5 결과 + 첫 Frame
- **출력**: KeyFrame(0) + N개의 MapPoint + 세계 좌표계 설정
- **파라미터**: `num_features_init` 임계값
- **가속 적용성**: 없음(로직 조립)
- **검증**: KeyFrame/MapPoint 수가 임계값 이상, 지도 포인트가 원점 주변 분포
- **시각화**: Three.js로 초기 포인트 클라우드 + 첫 카메라 frustum

### Step 7 — Frame Tracking (LK prev → curr)

- **학습 목표**: 투영 초기치를 활용한 LK 수렴 개선
- **C++ 원본**: `Frontend::TrackLastFrame`
- **입력**: 이전 Frame(+Features+MapPoints), 현재 Frame
- **출력**: 현재 Frame의 2D 특징점
- **파라미터**: 초기치 전략 토글 (map projection / 직전 위치 / 없음)
- **교체 가능 알고리즘**: Step4와 공유
- **가속 적용성**: Step4와 동일
- **검증**: 추적 성공 점 수 ≥ `num_features_tracking`
- **학습 포인트**: 초기치 전략별 수렴 라운드 수 비교 차트

### Step 8 — Pose Estimation (PnP via g2o)

- **학습 목표**: 반복 최적화 + 아웃라이어 제거의 상호작용
- **C++ 원본**: `Frontend::EstimateCurrentPose`, `g2o_types.h::EdgeProjectionPoseOnly`
- **입력**: 2D 관측점 + 대응 3D 점
- **출력**: 업데이트된 SE3 포즈, inlier 마스크
- **파라미터**: chi² 임계값(5.991 고정 시 체크박스), 반복 라운드 수(1–4), RobustKernel 제거 시점, δ(Huber)
- **교체 가능 알고리즘**:
  - g2o Iterative (원본)
  - OpenCV `solvePnPRansac` + Gauss-Newton
  - EPnP (closed-form)
  - DLS PnP
- **가속 적용성**: ✅ MT (엣지별 오차/야코비안), ✅ SIMD (행렬 연산), ✅ WebGPU (residual batch)
- **검증**: inlier 비율 > 70%, 연속 10프레임 포즈 궤적 스무딩 확인
- **시각화**: 각 라운드별 inlier 변화 애니메이션

### Step 9 — Keyframe Decision

- **학습 목표**: 키프레임 정책이 정확도·비용에 주는 영향
- **C++ 원본**: `Frontend::InsertKeyframe` 조건
- **입력**: 추적 inlier 수, 이전 KF와의 상대 운동
- **출력**: boolean + 이유
- **파라미터**: `num_features_needed_for_keyframe`
- **교체 가능 알고리즘**:
  - inlier count < 임계 (원본)
  - 이동 거리 / 회전 각 기반
  - 공분산 기반 (불확실성)
  - 하이브리드
- **가속 적용성**: 없음
- **검증**: 시퀀스에서 KF 비율이 5~15% 범위

### Step 10 — New MapPoints via Keyframe

- **학습 목표**: 새 KF에서 미추적 특징점을 어떻게 재삼각화하는가
- **C++ 원본**: `Frontend::TriangulateNewPoints`, `DetectFeatures`+`FindFeaturesInRight`
- **입력**: 새 KF의 left+right, 현재 포즈
- **출력**: 새 MapPoint 수
- **파라미터**: 새 특징점 비율 상한, depth 합리성 범위
- **교체 가능 알고리즘**: Step3·4·5와 공유 + "새 점만 재검출" vs "전량 재검출"
- **가속 적용성**: Step3·4·5와 동일
- **검증**: 새 KF당 추가 MapPoint 수 log 그래프

### Step 11 — Bundle Adjustment (Backend)

- **학습 목표**: 이항 엣지·Schur 보완·적응적 chi²
- **C++ 원본**: `Backend::Optimize`, `g2o_types.h::EdgeProjection`
- **입력**: active KF 창 + active landmarks
- **출력**: 업데이트된 포즈·랜드마크, 아웃라이어 관측 제거 수
- **파라미터**:
  - active window 크기(`num_active_keyframes` 3–15)
  - optimize 반복 수 (1–20)
  - chi² 초기 임계값
  - Schur 보완 on/off
- **교체 가능 알고리즘**:
  - g2o + CXSparse (원본 포팅)
  - g2o Dense
  - Levenberg-Marquardt (직접 구현 교육용 minimal)
- **가속 적용성**: ✅ MT (residual/Jacobian 병렬), ✅ SIMD, ✅ WebGPU (compute로 normal equation 빌드)
- **검증**: 평균 재투영 오차 감소율 ≥ 40%, 수렴 단조성
- **시각화**: 3D에서 최적화 전/후 오버레이 토글 + 오차 bar chart

### Step 12 — Map Management (Sliding Window)

- **학습 목표**: 중복 제거 vs 공간 다양성 트레이드오프
- **C++ 원본**: `Map::RemoveOldKeyframe`
- **입력**: 활성 KF 집합, 현재 KF
- **출력**: 제거 대상 KF, 그에 따른 active landmark 재계산
- **파라미터**: 중복 거리 임계값, 다양성 가중치
- **교체 가능 알고리즘**:
  - 중복 우선 → 최원거리 fallback (원본)
  - FIFO
  - Covisibility graph-based (고급)
- **가속 적용성**: 없음(로직)
- **검증**: 10KF 제거 후 평균 포즈 분산 확인

### Step 13 — Full Pipeline (End-to-End VO)

- **학습 목표**: 모든 단계를 연결해 실제 궤적을 그려본다
- **C++ 원본**: `app/run_kitti_stereo.cpp` + `VisualOdometry`
- **입력**: KITTI 전체(또는 mini) 시퀀스
- **출력**: 궤적(PLY export 가능), 포인트 클라우드
- **UI**: 재생/일시정지/프레임별 step-through, KITTI GT 궤적(있는 경우) 겹쳐 그리기
- **파라미터**: 이전 모든 단계의 파라미터를 프리셋으로 묶어 "conservative/aggressive/book-default" 3종 저장/불러오기
- **가속 적용성**: 메인 스레드 Frontend + 별도 Worker Backend + WebGL/WebGPU Viewer
- **검증**: 궤적이 원본 C++ 결과와 RMSE < 지정값 (샘플 시퀀스 기준 오프라인 측정)
- **학습 포인트**: Backend ON/OFF, sliding window size=1 등의 극단 설정을 시도해 시스템이 어떻게 붕괴하는지 관찰

---

## 4. 가속기 적용 매트릭스

|  | MT (pthreads) | WASM SIMD | WebGL (regl) | WebGPU (wgsl) |
|--|:-:|:-:|:-:|:-:|
| Step 3 Feature Detection | ✅ | ✅ | ✅ | ✅ |
| Step 4 Stereo LK | ✅ | ✅ | ✅ | ✅ |
| Step 5 Triangulation | ✅ | ✅ | — | ✅ |
| Step 7 Frame Tracking | ✅ | ✅ | ✅ | ✅ |
| Step 8 PnP | ✅ | ✅ | — | ✅ |
| Step 10 New MapPoints | ✅ | ✅ | ✅ | ✅ |
| Step 11 BA | ✅ | ✅ | — | ✅ |
| Step 13 Viewer | — | — | ✅ | ✅ |

- UI의 `AccelSwitch`에서 비활성 조합은 회색 처리하고 tooltip으로 이유를 설명한다.
- 각 가속 선택 시 브라우저/하드웨어 가용성(`navigator.hardwareConcurrency`, `crossOriginIsolated`, `navigator.gpu`)을 먼저 확인하고, 불가 시 자동 fallback + 배너 경고.

---

## 5. 공통 UI/UX 컨벤션

각 단계 화면은 아래 5영역 레이아웃을 따른다.

```
┌──────────────────────────────────────────────────────────────┐
│  Header: Step N · 제목 · 진행률 · C++ 소스 링크              │
├──────────────┬───────────────────────────┬───────────────────┤
│ Param Panel  │    Input Visualization    │ Output Visualiz.  │
│ (왼쪽 고정)  │    (가운데 이미지/3D)     │ (오른쪽 결과)     │
│ + Algo Picker│                           │                   │
│ + Accel Sw.  │                           │                   │
├──────────────┴───────────────────────────┴───────────────────┤
│  Perf Meter (ms/FPS/mem) · Benchmark Table · Diff Viewer     │
├──────────────────────────────────────────────────────────────┤
│  ✅ Verify Gate (체크리스트 통과 시 "다음 단계로" 활성화)    │
└──────────────────────────────────────────────────────────────┘
```

### 파라미터 정의 방식

`src/steps/StepXX/params.ts`에 JSON Schema-like 메타데이터로 선언하면 `ParamPanel`이 자동으로 UI를 생성한다.

```ts
export const params = {
  numFeatures: { type: 'int', min: 10, max: 500, default: 150, label: 'num_features' },
  detector:    { type: 'enum', options: ['GFTT','FAST','ORB','Harris'], default: 'GFTT' },
  accel:       { type: 'accel', allowed: ['cpu','cpu-simd','cpu-mt','cpu-mt-simd','webgl','webgpu'] },
} as const;
```

### 검증 게이트 (`VerifyGate`)

- 단계별 체크리스트 항목은 숫자 임계값 기반 자동 pass + 학습자 확인 체크박스(수동)의 혼합.
- 모두 통과해야 라우터가 다음 단계로 전환된다.
- 상태는 `pipelineStore`에 저장하고 `localStorage`로 영속화해 새로고침 시 복원.

### 벤치마크

- `benchmark.ts`: warm-up 3회 + 측정 10회의 `performance.now()` 기반 측정 + `performance.measureUserAgentSpecificMemory()` 지원 시 메모리 수집.
- 결과는 `(step, algo, accel)` 키로 `benchStore`에 누적 → uPlot 실시간 그래프.
- "Export CSV" 버튼으로 학습 결과를 내보낼 수 있다.

---

## 6. WASM 모듈 구조 & 바인딩

### 6.1 단계별 모듈 분리 방침

- 전체 파이프라인 모듈(`myslam-vo.wasm`) 1개 + 단계별 drill-down을 위한 **독립 노출 API**를 Embind로 제공한다.
- 가속 변형은 파일명으로 구분: `myslam-vo.baseline.wasm`, `myslam-vo.simd.wasm`, `myslam-vo.mt.wasm`, `myslam-vo.mt-simd.wasm`.
- 로더(`src/wasm/loader.ts`)가 선택된 가속 조합에 따라 해당 바이너리를 lazy-load한다.

### 6.2 Embind 인터페이스 예시

```cpp
// wasm-src/myslam/bindings/bind_features.cpp
EMSCRIPTEN_BINDINGS(features) {
  emscripten::class_<FeatureDetector>("FeatureDetector")
    .constructor<FeatureParams>()
    .function("detect", &FeatureDetector::detect)      // ImageData → vector<KeyPoint2D>
    .function("setParams", &FeatureDetector::setParams);
  emscripten::value_object<FeatureParams>("FeatureParams")
    .field("numFeatures", &FeatureParams::num_features)
    .field("minDistance", &FeatureParams::min_distance)
    .field("detectorKind", &FeatureParams::detector_kind); // enum
}
```

TS 측 타입은 `src/wasm/types.ts`에서 수동 선언 (dts-gen 대안). 바인딩 헤더 변경 시 CI에서 검증 스크립트 실행.

### 6.3 메모리 전달 규약

- 이미지: JS `Uint8ClampedArray` → `Module.HEAPU8.set()`로 복사 후 포인터 전달 (zero-copy 불가, 안전성 우선).
- 결과 배열(2D keypoints, 3D points): `emscripten::val` / `TypedArray view` 반환 후 JS에서 복사.
- 장기 상태 객체(Map, Frame 풀)는 WASM heap에 유지, JS는 핸들만 보유.

---

## 7. 빌드 파이프라인

### 7.1 스크립트 (package.json)

```json
{
  "scripts": {
    "dev": "vite",
    "build": "tsc -b && vite build",
    "preview": "vite preview",
    "build:wasm:baseline": "bash wasm-src/build.sh baseline",
    "build:wasm:simd":     "bash wasm-src/build.sh simd",
    "build:wasm:mt":       "bash wasm-src/build.sh mt",
    "build:wasm:mt-simd":  "bash wasm-src/build.sh mt-simd",
    "build:wasm":          "npm-run-all build:wasm:baseline build:wasm:simd build:wasm:mt build:wasm:mt-simd",
    "setup:emsdk":         "bash scripts/setup-emsdk.sh"
  }
}
```

### 7.2 Emscripten 플래그 프로파일

| variant | 추가 플래그 |
|---------|-----------|
| baseline | `-O3 -sMODULARIZE=1 -sEXPORT_ES6=1 -sALLOW_MEMORY_GROWTH=1 --bind` |
| simd | + `-msimd128 -DEIGEN_VECTORIZE` |
| mt | + `-pthread -sUSE_PTHREADS=1 -sPTHREAD_POOL_SIZE=navigator.hardwareConcurrency` |
| mt-simd | 위 둘 조합 |

모든 변형 공통: `-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPU8','HEAPF32']`, `-sWASM=1`.

### 7.3 Vite 설정 포인트

`vite.config.ts`에서 COEP/COOP 헤더를 dev server와 preview에 모두 설정하고, `worker.format: 'es'`로 워커 번들을 보장한다.

```ts
server: {
  headers: {
    'Cross-Origin-Opener-Policy': 'same-origin',
    'Cross-Origin-Embedder-Policy': 'require-corp',
  },
},
optimizeDeps: { exclude: ['*.wasm'] },
worker: { format: 'es' },
```

### 7.4 CI (GitHub Actions 권장)

- matrix: `[baseline, simd, mt, mt-simd]` × `[ubuntu-latest]`
- 각 variant의 `.wasm` 크기·SHA256·빌드 시간 리포트.
- Playwright 스모크: 각 Step 화면이 로드되고 검증 게이트를 자동 프리셋으로 통과하는지 테스트.

---

## 8. 데이터 준비

1. KITTI odometry sequence 05에서 **처음 10프레임만** 추출해 `public/datasets/kitti05-mini/`에 포함.
2. 이미지는 **AVIF 또는 WebP lossless**로 재인코딩해 용량을 < 10MB로 제한(오리지널 PNG는 너무 큼). JPEG-XL은 Chrome이 지원을 중단했으므로 사용하지 않는다.
3. `calib.txt`는 원본 그대로 유지 (파서 학습 목적).
4. "더 긴 시퀀스"는 첫 실행 시 사용자 동의 받은 후 CDN에서 prefetch (선택).
5. GT 궤적은 포함하되 optional — Step13의 오차 비교에 사용.

---

## 9. 구현 단계별 작업 지시 (로드맵)

> 각 Phase 종료 시 **PR 단위 커밋** + **작동하는 데모** + **학습 노트 MD** 3종을 동시에 산출한다.

### Phase A — 프로젝트 스캐폴딩 (0.5주)
1. Vite + React + TS 스캐폴드 생성 (`ch13-wasm/` 디렉토리).
2. COEP/COOP 헤더 + 공통 레이아웃 + 라우터(react-router) + Zustand 스토어.
3. `wasm-src/` 하위에 Emscripten SDK 준비 스크립트, CMake toolchain, `hello_world.cpp` 바인딩으로 **엔드투엔드 WASM 로딩** 확인.
4. 공통 컴포넌트(`StepLayout`, `PerfMeter`, `VerifyGate`, `ParamPanel`) skeleton 구현.

### Phase A+ — g2o/CXSparse WASM 스파이크 (0.5~1주) ⚠️ 가장 중요

**목표**: Phase B 이후 일정을 확정하기 전에, g2o+Eigen+CXSparse의 Emscripten 빌드가 실제로 가능한지 **시간 제한 스파이크**로 검증한다.

1. g2o `core` + `solvers/csparse` + `types/sba`의 최소 set을 Emscripten으로 빌드.
2. 10~20행 수준의 **PnP 예제**를 Embind로 노출해 JS에서 호출.
3. 결과가 OpenCV `solvePnPRansac`과 수치적으로 일치하는지 확인.

**Gate 조건**:
- ✅ 성공 → Phase G(번들 조정)까지 원안대로 진행.
- ❌ 실패(빌드/런타임/성능) → 전체 계획을 **경량 직접 구현 LM(Levenberg-Marquardt) 기반**으로 피벗한다. Eigen + Sophus만으로 작성한 ~500줄 BA 최소 구현을 대체 경로로 채택하고, Step 8/11의 "교체 가능 알고리즘" 목록에서 g2o 항목을 제거한다.

이 스파이크 결과 없이 Phase B~H의 일정과 구현 순서를 확정하지 않는다.

### Phase B — 단계 1~2 (Dataset + Camera, 0.5주)
1. KITTI 경량 샘플 준비 + 파서(TS) + WASM Camera 바인딩.
2. Step1/Step2 화면 완성 + 게이트 통과 확인.

### Phase C — 특징 & 스테레오 (Step 3~4, 1.5주)
1. OpenCV.js minimal 빌드.
2. GFTT/LK WASM 경로 + WebGL Harris · WebGPU GFTT 대체 구현.
3. MT/SIMD variant 빌드로 벤치마크 비교.
4. 알고리즘 교체 UI(`AlgoPicker`) 완성.

### Phase D — 삼각화·초기 맵 (Step 5~6, 1주)
1. Eigen SVD WASM 포팅 + 알고리즘 3종.
2. Three.js Scene3D 컴포넌트 + 초기 포인트 클라우드 시각화.
3. 학습용 "반환 반전 버그" 토글 재현.

### Phase E — 추적·PnP (Step 7~8, 1.5주)
1. g2o 최소 포팅(또는 직접 구현 최소 LM) + EdgeProjectionPoseOnly.
2. OpenCV solvePnPRansac 경로 포함.
3. 벤치 & 수렴 라운드 시각화.

### Phase F — 키프레임·신규 맵포인트 (Step 9~10, 0.5주)
1. 정책 3~4종.
2. Step3/4/5 재사용.

### Phase G — 번들 조정 (Step 11, 2주) — 가장 난이도 높음
1. g2o (또는 대체) BA 포팅.
2. Worker 분리 + condition_variable 유사 동기화(Atomics).
3. MT/SIMD/WebGPU 경로.
4. 적응적 chi² 로직 시각화.

### Phase H — 슬라이딩 윈도우 & 풀 파이프라인 (Step 12~13, 1주)
1. `Map` 매니저 포팅.
2. 전체 VO 워커 + Three.js 실시간 궤적.
3. 프리셋·내보내기·GT 비교.

### Phase I — 마감 (0.5주)
1. a11y, 모바일 레이아웃, dark mode.
2. 문서(`docs/steps/*.md`) 완성.
3. Playwright 스모크 + 퍼포먼스 회귀 가드.

> **총 예상: 9~14주 (1인 풀타임 기준)**. Phase A+ 스파이크 결과에 따라 Phase G가 2주(g2o 포팅 성공)에서 4~5주(직접 구현 LM 피벗)로 변동할 수 있다. 실제 단계 착수 시마다 Phase 카드를 세부 작업으로 분해할 것.

---

## 10. 비기능 요구사항

- **성능**: Step 13 풀 파이프라인이 1개 스레드 baseline에서 10 FPS 이상, mt-simd에서 30 FPS 이상 (mini dataset 기준).
- **초기 로드**: 첫 화면 TTI ≤ 3s (WASM lazy load 유지).
- **번들**: JS 초기 페이로드 ≤ 400KB gzipped (WASM 제외).
- **접근성**: 주요 컨트롤 키보드 조작 지원, 색맹 친화 팔레트.
- **다국어**: 1차는 한국어, 라벨은 `src/i18n/ko.ts` 구조로 준비해 추후 en 추가 가능.
- **라이선스 고지**: OpenCV, g2o, Eigen, Sophus, CXSparse 각 라이선스 `public/LICENSES.txt`에 명시.

---

## 11. 위험 요소와 완화

| 위험 | 영향 | 완화책 |
|------|------|--------|
| g2o WASM 포팅 난항 | Step 8/11 지연, 계획 전반 재설계 | **Phase A+ 스파이크로 선행 검증**(§9). 실패 시 직접 구현 minimal LM으로 피벗. OpenCV `solvePnPRansac`은 항상 유지 |
| GitHub Pages에서 COEP/COOP 설정 불가 | MT variant 배포 불가 | 주 배포는 Cloudflare Pages / Vercel / Netlify로. GH Pages 필요 시 service-worker COEP shim 또는 MT variant 비활성 빌드 제공 |
| OpenCV.js 용량 | 초기 로드 저하 | GFTT/LK/ORB만 포함하는 커스텀 빌드 + code splitting |
| SharedArrayBuffer 미지원 환경 | MT variant 불가 | `crossOriginIsolated` 감지 후 자동 CPU single-thread로 fallback |
| WebGPU 호환성 | 일부 브라우저 실패 | feature-detect 후 WebGL로 대체, 기능 비활성 배지 표시 |
| KITTI 데이터 용량 | 첫 로드 무거움 | 10프레임 mini + 나머지는 on-demand prefetch |
| SVD 수치 안정성 | 삼각화 실패 | 조건수 표시 + 알고리즘 교체 UI로 학습자에게 원인 이해 유도 |
| Emscripten pthreads의 deadlock | Step 11 불안정 | Atomics.waitAsync 사용, Worker에 timeout/heartbeat |

---

## 12. 산출물 정의(DoD)

이 계획이 모두 완료되면 다음을 만족한다.

1. `npm run dev`로 로컬에서 13개 교육 화면을 모두 탐색할 수 있다.
2. 각 단계에서 **최소 2개의 교체 알고리즘**과 **최소 2개의 가속 모드**를 선택해 결과와 성능 차이를 확인할 수 있다(해당되는 단계).
3. 검증 게이트가 설계된 순서로 통과되며, 통과 상태는 `localStorage`에 영속화된다.
4. 풀 파이프라인(Step 13)이 KITTI mini 시퀀스에서 원본 C++ 구현의 궤적과 RMSE < 0.5m 이내로 일치한다.
5. **COEP/COOP 헤더를 설정할 수 있는 호스팅(Cloudflare Pages, Vercel, Netlify 등)**에 정적 배포 가능한 빌드 산출물을 생성한다. GitHub Pages는 COEP/COOP 설정이 불가능해 MT variant를 구동할 수 없으므로 **주 배포 타깃에서 제외**한다(GitHub Pages에 올리는 경우, service-worker 기반 COEP shim을 포함하는 별도 빌드를 옵션으로 제공하거나 MT variant를 비활성화한다).

---

## 13. 참고 자료

- 본 프로젝트 분석: `docs/analysis/2026-04-17-ch13-full-analysis.md`
- slambook2 ch13 원본 코드: `ch13/` 디렉토리
- Emscripten pthreads: https://emscripten.org/docs/porting/pthreads.html
- WASM SIMD: https://github.com/WebAssembly/simd
- OpenCV.js 빌드: https://docs.opencv.org/4.x/d4/da1/tutorial_js_setup.html
- g2o: https://github.com/RainerKuemmerle/g2o
- WebGPU Best Practices: https://toji.dev/webgpu-best-practices/

---

## 부록 A — 단계 카드 체크리스트 템플릿

각 단계 구현 시 `docs/steps/NN-xxx.md`에 아래 템플릿을 채운다.

```markdown
# Step NN — 제목

## 학습 목표
- [ ] ...

## C++ 원본 매핑
- 파일: `ch13/src/xxx.cpp:함수명`

## UI
- [ ] ParamPanel: ...
- [ ] AlgoPicker: ...
- [ ] AccelSwitch: ...
- [ ] Input View: ...
- [ ] Output View: ...
- [ ] PerfMeter 통합
- [ ] VerifyGate 체크리스트

## 알고리즘
- [ ] A (원본)
- [ ] B (대체)
- [ ] C (학습용 단순)

## 가속 경로
- [ ] CPU scalar
- [ ] CPU SIMD
- [ ] CPU MT
- [ ] WebGL (해당 시)
- [ ] WebGPU (해당 시)

## 검증
- [ ] 자동 수치 게이트
- [ ] 시각 수동 게이트
- [ ] 원본 C++ 결과와의 diff

## 벤치마크
- [ ] baseline vs simd vs mt vs mt-simd 4행 표
- [ ] 알고리즘 교차 비교

## 학습 노트
- 본문(~500자)
- "의도된 실패" 시나리오 설명
```

— 끝.
