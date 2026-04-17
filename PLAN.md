# PLAN: slambook2 ch13 스테레오 VO 교육용 WebAssembly 프로젝트

> **분석 기준 문서**: `docs/analysis/2026-04-17-ch13-full-analysis.md`  
> **목적**: ch13 myslam 파이프라인을 단계별 교육용 WebAssembly 앱으로 구현

---

## 프로젝트 개요

- **작업 디렉토리**: `/Users/swcho/projects/spatial-ai/wasm-workshop/slam-vo-edu/`
- **프론트엔드**: Vite + React + TypeScript
- **WASM 빌드**: Emscripten (emcc)
- **핵심 C++ 의존**: OpenCV (wasm 빌드), Eigen3, Sophus, 경량 g2o 대체 최적화기
- **렌더링**: WebGL (Three.js) → WebGPU (선택)
- **병렬화**: Emscripten pthreads (SharedArrayBuffer) + WASM SIMD

---

## 구현 단계 (8개 Phase)

| Phase | 알고리즘 | 입력 | 출력 | 가속화 옵션 |
|-------|---------|------|------|------------|
| 1 | 데이터셋 로딩 | KITTI 이미지 파일 | Frame(left, right) 썸네일 | - |
| 2 | 특징점 검출 | 그레이스케일 이미지 | 키포인트 시각화 | SIMD |
| 3 | 스테레오 LK 매칭 | left+right 이미지 | 매칭 시각화 | SIMD, MT |
| 4 | 스테레오 삼각화 | 매칭 쌍 + 카메라 파라미터 | 3D 포인트 클라우드 | SIMD |
| 5 | 프레임간 LK 추적 | 이전/현재 left 이미지 | 추적 벡터 시각화 | SIMD, MT |
| 6 | PnP 포즈 추정 | 3D-2D 대응점 | 카메라 SE3 포즈 | - |
| 7 | 번들 조정 (BA) | 활성 키프레임 창 | 최적화된 포즈+랜드마크 | MT, WebGPU |
| 8 | 3D 맵 시각화 | 포즈 궤적 + 랜드마크 | 실시간 3D 뷰 | WebGL, WebGPU |

---

## Step 0 — 프로젝트 초기화

### 프롬프트

```
다음 경로에 Vite + React + TypeScript 프로젝트를 생성해 줘:
/Users/swcho/projects/spatial-ai/wasm-workshop/slam-vo-edu/

요구사항:
- npm create vite@latest 로 생성 (react-ts 템플릿)
- 추가 패키지: three, @types/three, tailwindcss, zustand, immer
- 디렉토리 구조:
  slam-vo-edu/
  ├── src/
  │   ├── components/          # React UI 컴포넌트
  │   │   ├── Pipeline/        # 파이프라인 단계별 패널
  │   │   ├── Controls/        # 파라미터 컨트롤
  │   │   └── Viewer3D/        # WebGL/WebGPU 3D 뷰어
  │   ├── wasm/                # WASM 바인딩 및 로더
  │   │   ├── loader.ts        # emscripten 모듈 로드
  │   │   └── bindings.ts      # C++ 함수 타입 정의
  │   ├── store/               # Zustand 전역 상태
  │   ├── workers/             # Web Worker 파일들
  │   └── types/               # 공통 TypeScript 타입
  ├── cpp/                     # C++ WASM 소스
  │   ├── CMakeLists.txt
  │   ├── core/                # 알고리즘 구현
  │   └── bindings/            # Emscripten 바인딩
  ├── public/
  │   └── wasm/                # 빌드된 .wasm/.js 파일 배치
  └── scripts/
      └── build_wasm.sh        # WASM 빌드 스크립트

vite.config.ts에 다음을 설정해 줘:
- COOP/COEP 헤더 (SharedArrayBuffer 활성화)
- /wasm/ 경로 정적 파일 서빙
- worker 포맷: module 타입

기본 레이아웃:
- 왼쪽 사이드바: 파이프라인 단계 목록 (Phase 1~8)
- 중앙: 현재 단계의 입/출력 시각화 패널
- 오른쪽: 파라미터 컨트롤 패널 + 퍼포먼스 대시보드
- 하단: 단계 완료 체크 + 다음 단계 진행 버튼
```

### 검증 기준
- [ ] `npm run dev` 실행 시 기본 레이아웃 렌더링
- [ ] `SharedArrayBuffer is defined` 콘솔 확인
- [ ] `/public/wasm/` 디렉토리 접근 가능

---

## Step 1 — Emscripten WASM 빌드 환경 구성

### 프롬프트

```
slam-vo-edu/cpp/ 디렉토리에 Emscripten 기반 WASM 빌드 환경을 구성해 줘.

요구사항:
1. CMakeLists.txt 작성:
   - CMAKE_TOOLCHAIN_FILE = $EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake
   - target_compile_options: -msimd128, -pthread (선택적 빌드 타겟별)
   - target_link_options:
     * -sWASM=1
     * -sEXPORT_ES6=1
     * -sMODULARIZE=1
     * -sENVIRONMENT=web,worker
     * -sUSE_PTHREADS=1 (mt 타겟)
     * -sPTHREAD_POOL_SIZE=4
     * -sINITIAL_MEMORY=256MB
     * -sALLOW_MEMORY_GROWTH=1
     * --bind (Embind)

2. 의존성 통합:
   - Eigen3: 헤더 전용, FetchContent로 다운로드
   - Sophus: 헤더 전용, FetchContent로 다운로드 (Eigen 의존)
   - OpenCV: opencv-wasm 사전 빌드 바이너리 사용
     * npm 패키지 opencv-wasm 또는 직접 wasm 빌드
     * 필요 모듈: core, imgproc, features2d, video (optflow)
   - Ceres 또는 경량 LM 최적화기: 자체 구현 (g2o 대체)
     * g2o의 WASM 빌드는 복잡도 높음 → 교육용 경량 LM optimizer 직접 구현

3. 빌드 타겟 3종:
   - slam_core_st.js: 단일 스레드 (기본)
   - slam_core_mt.js: 멀티스레드 (-sUSE_PTHREADS)
   - slam_core_simd.js: SIMD 활성화 (-msimd128)

4. scripts/build_wasm.sh 작성:
   - emsdk 환경 활성화
   - 3종 타겟 순차 빌드
   - 출력 파일을 public/wasm/로 복사

cpp/bindings/ 아래에 Embind 바인딩 파일을 작성해 줘:
   - bind_feature_detection.cpp: GFTT, FAST 특징점 검출
   - bind_optical_flow.cpp: LK 광학흐름 (calcOpticalFlowPyrLK 래핑)
   - bind_triangulation.cpp: SVD 삼각화 (algorithm.h 포팅)
   - bind_pnp.cpp: PnP 포즈 추정 (경량 LM)
   - bind_bundle_adjustment.cpp: 슬라이딩 윈도우 BA
```

### 검증 기준
- [ ] `bash scripts/build_wasm.sh` 성공
- [ ] `public/wasm/slam_core_st.js` 생성 확인
- [ ] `public/wasm/slam_core_mt.js` 생성 확인
- [ ] 브라우저 콘솔에서 `SlamModule()` 로드 확인

---

## Step 2 — WASM 로더 및 타입 바인딩

### 프롬프트

```
src/wasm/ 디렉토리에 WASM 모듈 로더와 TypeScript 바인딩을 작성해 줘.

요구사항:
1. src/wasm/loader.ts:
   - 싱글톤 패턴으로 모듈 로드 (중복 로드 방지)
   - 3종 빌드(st/mt/simd) 런타임 교체 지원
   - 로드 진행률 콜백 제공
   - 예시:
     ```typescript
     type BuildVariant = 'st' | 'mt' | 'simd';
     class SlamModuleLoader {
       static async load(variant: BuildVariant): Promise<SlamModule>
       static getActive(): SlamModule | null
       static switch(variant: BuildVariant): Promise<void>
     }
     ```

2. src/wasm/bindings.ts (TypeScript 인터페이스):
   ```typescript
   interface KeyPoint { x: number; y: number; response: number; }
   interface OpticalFlowResult { points: Float32Array; status: Uint8Array; }
   interface Point3D { x: number; y: number; z: number; }
   interface SE3Pose { rotation: Float64Array; // 3x3, row-major; translation: Float64Array; }
   
   interface SlamModule {
     // Phase 2: 특징점 검출
     detectGFTT(imgData: Uint8Array, width: number, height: number,
                maxCorners: number, qualityLevel: number, minDistance: number): KeyPoint[];
     detectFAST(imgData: Uint8Array, width: number, height: number,
                threshold: number): KeyPoint[];
     
     // Phase 3 & 5: 광학 흐름
     calcOpticalFlow(prevImg: Uint8Array, currImg: Uint8Array,
                     prevPts: Float32Array, width: number, height: number,
                     winSize: number, maxLevel: number): OpticalFlowResult;
     
     // Phase 4: 삼각화
     triangulate(projMat0: Float64Array, projMat1: Float64Array,
                 pts0: Float32Array, pts1: Float32Array): Point3D[];
     
     // Phase 6: PnP
     solvePnPIterative(points3d: Float64Array, points2d: Float64Array,
                       K: Float64Array, distCoeffs: Float64Array,
                       maxIter: number, chiThreshold: number): SE3Pose;
     
     // Phase 7: 번들 조정
     bundleAdjust(poses: Float64Array, landmarks: Float64Array,
                  observations: Int32Array, obsPoints: Float64Array,
                  K: Float64Array, maxIter: number): { poses: Float64Array; landmarks: Float64Array; };
   }
   ```

3. src/workers/slamWorker.ts:
   - Web Worker 내에서 WASM 모듈 실행
   - Comlink 또는 직접 postMessage 프로토콜
   - 각 Phase 함수를 비동기로 메인 스레드에 노출
```

### 검증 기준
- [ ] TypeScript 컴파일 오류 없음
- [ ] Worker에서 WASM 로드 후 `detectGFTT` 호출 성공
- [ ] 콘솔에 `WASM module loaded (variant: st)` 출력

---

## Step 3 — Phase 1: 데이터셋 로딩 UI

### 프롬프트

```
src/components/Pipeline/Phase1Dataset.tsx 를 구현해 줘.

기능:
1. 사용자가 KITTI 시퀀스 디렉토리(또는 ZIP 파일)를 드래그&드롭 또는 파일 선택으로 업로드
2. calib.txt 파싱:
   - P0, P1 투영 행렬(3×4) 파싱
   - fx, fy, cx, cy, baseline 추출 (P0[0,0], P0[1,1], P0[0,2], P0[1,2], -P1[0,3]/fx)
3. image_0/, image_1/ 디렉토리에서 이미지 파일 목록 로드
4. UI 구성:
   - 좌: calib.txt 내용 + 파싱된 카메라 파라미터 테이블
   - 우: 첫 프레임 left/right 이미지 썸네일 (나란히)
   - 하단: 총 프레임 수, 이미지 해상도, 업로드된 파일 트리
5. 상태 관리: Zustand store에 저장
   ```typescript
   interface DatasetState {
     frames: { left: ImageData; right: ImageData }[];
     camera: { fx, fy, cx, cy, baseline, P0, P1: number[][] };
     currentFrameIdx: number;
   }
   ```
6. "Phase 1 완료 → Phase 2로 진행" 버튼 (카메라 파라미터 + 최소 10프레임 로드 시 활성화)

참고:
- KITTI 이미지는 0.5배 다운샘플링 (Dataset.cpp 참조)
- 실제 KITTI 데이터 없이 테스트할 수 있도록 합성 스테레오 이미지 생성기도 제공:
  * 랜덤 체커보드 패턴 + 알려진 카메라 파라미터로 테스트 프레임 생성
```

### 검증 기준
- [ ] 파일 드롭 후 카메라 파라미터 테이블 렌더링
- [ ] 좌/우 이미지 썸네일 표시
- [ ] Zustand store에 camera 상태 저장 확인
- [ ] 합성 이미지 생성기로 KITTI 없이 동작 확인

---

## Step 4 — Phase 2: 특징점 검출 UI

### 프롬프트

```
src/components/Pipeline/Phase2FeatureDetection.tsx 를 구현해 줘.

기능:
1. 알고리즘 선택 탭: GFTT (GoodFeaturesToTrack) | FAST
2. 파라미터 컨트롤 (실시간 슬라이더):
   - GFTT: maxCorners(50~500), qualityLevel(0.001~0.1), minDistance(5~30)
   - FAST: threshold(5~50), nonmaxSuppression(체크박스)
3. 이미지 캔버스에 검출된 키포인트 오버레이 (색: response 값에 따라 열화상)
4. 통계 패널: 검출 수, 평균 response, 처리 시간(ms)
5. 퍼포먼스 비교 패널:
   - [단일스레드] [SIMD] 버튼으로 각각 실행 후 ms 비교 막대 그래프
6. 좌측: 원본 이미지 | 우측: 키포인트 오버레이
7. 파라미터를 바꿀 때마다 자동 재실행 (debounce 200ms)

C++ 구현 주의사항:
- WASM 바인딩에서 ImageData(RGBA)를 받아 그레이스케일 변환 후 처리
- GFTT는 frontend.cpp:DetectFeatures() 로직 참조
  * goodFeaturesToTrack + 격자(grid) 기반 균등 분포

완료 조건: maxCorners 이상의 특징점 검출 시 "Phase 2 완료" 활성화
```

### 검증 기준
- [ ] GFTT/FAST 탭 전환 시 다른 파라미터 슬라이더 표시
- [ ] 슬라이더 조작 시 캔버스 실시간 업데이트
- [ ] st vs SIMD 처리 시간 비교 그래프 표시
- [ ] 검출 수가 `num_features_init`(기본 50) 이상 시 완료 버튼 활성화

---

## Step 5 — Phase 3: 스테레오 LK 매칭 UI

### 프롬프트

```
src/components/Pipeline/Phase3StereoMatching.tsx 를 구현해 줘.

기능:
1. 좌측 키포인트 → 우측 이미지로 LK 광학흐름 매칭
   - frontend.cpp:FindFeaturesInRight() 로직 포팅
   - 맵포인트 투영 초기 추정치 활용 옵션 (체크박스)
2. 파라미터 컨트롤:
   - winSize: 7~31 (홀수)
   - maxLevel: 1~5 (피라미드 레벨)
   - 최소 dispariy 임계값 (에피폴라 제약: 동일 행 기준)
3. 시각화:
   - 좌/우 이미지를 수평으로 나란히 배치
   - 성공 매칭: 초록 선 / 실패: 빨간 점
   - 에피폴라 라인 토글 (수평선 오버레이)
4. 매칭 통계: 전체 시도 / 성공 / 성공률(%) / 평균 disparity
5. 퍼포먼스 비교: [ST] [MT 2쓰레드] [MT 4쓰레드] [SIMD] 버튼

완료 조건: 성공 매칭 수 >= num_features_init (50) 시 완료
```

### 검증 기준
- [ ] 좌/우 이미지에 매칭 라인 오버레이
- [ ] 에피폴라 라인 토글 동작
- [ ] MT 옵션 전환 시 SharedArrayBuffer 사용 확인
- [ ] 4가지 실행 모드 시간 비교 그래프

---

## Step 6 — Phase 4: 삼각화 UI

### 프롬프트

```
src/components/Pipeline/Phase4Triangulation.tsx 를 구현해 줘.

기능:
1. 스테레오 매칭 결과(좌/우 대응점)와 카메라 파라미터로 3D 포인트 생성
   - algorithm.h:triangulation() SVD 방식 포팅
   - 원서 버그 수정: σ₄/σ₃ < 0.01 이면 실패(false) 반환이 올바름
2. 파라미터 컨트롤:
   - depthMin / depthMax 필터 (Z > 0, Z < maxDepth)
   - SVD 품질 임계값 (σ₄/σ₃ ratio slider)
3. 3D 시각화 (Three.js Points):
   - 카메라 좌표계 기준 포인트 클라우드
   - 색상: depth 값에 따른 컬러맵 (viridis)
   - 마우스 드래그로 회전, 휠로 줌
   - 카메라 좌표축 (RGB 화살표) 표시
4. 통계: 삼각화 성공/실패 수, Z 분포 히스토그램
5. WebGL vs WebGPU 렌더링 토글 (포인트 클라우드 렌더링 한정)

완료 조건: 성공적으로 삼각화된 3D 포인트 >= 50개
```

### 검증 기준
- [ ] Three.js 캔버스에 포인트 클라우드 렌더링
- [ ] 깊이 컬러맵 적용
- [ ] 마우스 인터랙션 동작
- [ ] WebGL/WebGPU 토글 전환

---

## Step 7 — Phase 5: 프레임간 추적 UI

### 프롬프트

```
src/components/Pipeline/Phase5Tracking.tsx 를 구현해 줘.

기능:
1. 이전 프레임 left → 현재 프레임 left로 LK 추적
   - frontend.cpp:TrackLastFrame() 로직 포팅
   - 맵포인트 투영 초기 추정치 사용 옵션
2. 프레임 시퀀서 UI:
   - 이전/현재 프레임 썸네일 나란히
   - 프레임 슬라이더 (수동 스텝)
   - 자동 재생 버튼 (1~30 fps 선택)
3. 시각화:
   - 현재 프레임에 추적 화살표 오버레이 (이전 위치 → 현재 위치)
   - 성공 추적: 초록 / 실패: 빨간 X
   - 추적 이력 궤적 (fade-out 효과)
4. 상태 표시: TRACKING_GOOD / TRACKING_BAD / LOST 뱃지
   - 기준: frontend.h 파라미터 (num_features_tracking=50, bad=20)
5. 퍼포먼스 비교: [ST] [SIMD] [MT] 비교

완료 조건: 5프레임 연속 TRACKING_GOOD 달성
```

### 검증 기준
- [ ] 자동 재생 모드에서 추적 화살표 업데이트
- [ ] TRACKING_BAD 상태에서 배경색 변화
- [ ] 추적 이력 궤적 fade-out 효과

---

## Step 8 — Phase 6: PnP 포즈 추정 UI

### 프롬프트

```
src/components/Pipeline/Phase6PoseEstimation.tsx 를 구현해 줘.

기능:
1. 3D-2D 대응점으로 카메라 포즈(SE3) 추정
   - frontend.cpp:EstimateCurrentPose() 4-라운드 반복 최적화 포팅
   - 경량 LM (Levenberg-Marquardt) 옵티마이저 직접 구현 (g2o 대체)
   - chi² 기반 아웃라이어 마킹 + 3라운드 후 RobustKernel 제거

2. 파라미터 컨트롤:
   - 최적화 라운드 수: 1~10
   - chi² 임계값: 3.0~15.0 (기본 5.991 = χ²(2, 0.95))
   - RobustKernel 제거 라운드: 1~5

3. 시각화:
   - 재투영 오차 scatter plot (최적화 전/후 비교)
   - 포즈 SE3 행렬 표시 (4×4)
   - 아웃라이어/인라이어 분류 현황 (파이 차트)
   - 반복별 비용(cost) 수렴 그래프

4. 3D 뷰어에 포즈 궤적 누적 표시 (Phase 4 뷰어 재사용)

완료 조건: inlier 수 >= num_features_tracking (50)
```

### 검증 기준
- [ ] 최적화 라운드별 cost 그래프 표시
- [ ] 아웃라이어/인라이어 파이 차트
- [ ] 3D 뷰어에 포즈 화살표 추가

---

## Step 9 — Phase 7: 번들 조정 UI

### 프롬프트

```
src/components/Pipeline/Phase7BundleAdjustment.tsx 를 구현해 줘.

기능:
1. 슬라이딩 윈도우 번들 조정
   - backend.cpp:Optimize() 포팅 (경량 LM 사용)
   - VertexPose(SE3, 6-DOF) + VertexXYZ(3D) 동시 최적화
   - Schur 보완 적용 (랜드마크 marginalization)
   - 아웃라이어 비율 > 50% 시 chi² 임계값 ×2 (최대 5회) 적응 로직

2. 파라미터 컨트롤:
   - 활성 키프레임 창 크기: 3~10 (기본 7, map.h 참조)
   - 최적화 반복 수: 5~50 (기본 10)
   - chi² 임계값, 적응 최대 횟수

3. 시각화:
   - BA 전/후 포즈 궤적 오버레이 (다른 색)
   - BA 전/후 랜드마크 분포 비교
   - 반복별 전체 비용 수렴 곡선 (실시간 업데이트)
   - 아웃라이어 제거 비율 표시

4. 퍼포먼스 비교:
   - [ST] [MT: Schur 병렬] [WebGPU: 행렬 연산 가속] 버튼
   - WebGPU 경로: 재투영 잔차 계산을 Compute Shader로 가속

완료 조건: BA 후 평균 재투영 오차 < 2.0px
```

### 검증 기준
- [ ] BA 전/후 궤적 비교 시각화
- [ ] 비용 수렴 그래프 실시간 업데이트
- [ ] MT 모드에서 처리 시간 단축 확인
- [ ] WebGPU Compute Shader 실행 확인

---

## Step 10 — Phase 8: 전체 3D 맵 뷰어

### 프롬프트

```
src/components/Viewer3D/MapViewer.tsx 를 구현해 줘.

기능:
1. 전체 파이프라인 실행 결과를 3D로 시각화
   - 카메라 궤적 (연속 라인, 키프레임은 큰 점)
   - 랜드마크 포인트 클라우드
   - 현재 카메라 뷰 프러스텀 (와이어프레임)
   - 활성 윈도우 키프레임 강조 표시

2. 렌더링 모드 선택:
   - [WebGL - Three.js]: 즉시 사용 가능 기본 모드
   - [WebGPU - 네이티브]: Three.js WebGPU renderer 사용
   - 포인트 수 > 10만: WebGPU 권장 알림

3. 인터랙션:
   - 마우스 드래그: 궤도 회전 (OrbitControls)
   - 우클릭: 팬
   - 휠: 줌
   - 더블클릭: 해당 키프레임으로 시점 이동

4. 오버레이 통계:
   - 총 키프레임 수, 랜드마크 수
   - 현재 카메라 위치 (x, y, z)
   - 누적 이동 거리

5. 스크린샷 / GLB 내보내기 버튼

WebGPU 구현:
- GPUBuffer로 포인트 데이터 직접 관리
- Compute Shader로 뷰 프러스텀 컬링 수행
- 동적 포인트 수 변화에 대응하는 버퍼 재할당 전략
```

### 검증 기준
- [ ] WebGL 모드에서 궤적 + 포인트 클라우드 렌더링
- [ ] WebGPU 모드 전환 및 렌더링
- [ ] OrbitControls 인터랙션
- [ ] GLB 내보내기 동작

---

## Step 11 — 퍼포먼스 대시보드

### 프롬프트

```
src/components/Controls/PerformanceDashboard.tsx 를 구현해 줘.

기능:
1. 각 Phase별 실행 시간 누적 테이블:
   | Phase | ST(ms) | SIMD(ms) | MT(ms) | WebGL(ms) | WebGPU(ms) |
   |-------|--------|----------|--------|-----------|------------|
   * 각 Phase를 실행할 때마다 자동 갱신

2. 실시간 FPS 미터 (전체 파이프라인 연속 실행 모드)

3. 가속화 조합 선택기 (체크박스 그룹):
   - [ ] SIMD (WASM SIMD128)
   - [ ] Multi-thread (pthreads, 2/4/8 workers)
   - [ ] WebGL (Three.js renderer)
   - [ ] WebGPU (Compute Shader)
   선택한 조합으로 "벤치마크 실행" → 각 Phase 5회 평균

4. 브라우저 호환성 경고:
   - SharedArrayBuffer 미지원: MT 비활성화
   - WebGPU 미지원: WebGPU 비활성화
   - WASM SIMD 미지원: SIMD 비활성화

5. 결과 CSV 내보내기
```

### 검증 기준
- [ ] 각 Phase 실행 시 자동 시간 기록
- [ ] 벤치마크 실행 후 조합별 평균 시간 표시
- [ ] 브라우저 기능 감지 및 비활성화 처리

---

## Step 12 — 파이프라인 통합 실행

### 프롬프트

```
src/components/Pipeline/PipelineRunner.tsx 를 구현해 줘.

기능:
1. "전체 실행" 버튼: Phase 1~8을 순서대로 자동 실행
2. 각 Phase 완료 시 사이드바 아이콘 업데이트 (⏳→✅)
3. 특정 Phase에서 실패 시 중단 + 오류 메시지 + 해당 Phase로 포커스 이동
4. 스텝 모드: 한 Phase씩 수동으로 진행 (교육 목적)
5. 진행률 표시바 (전체 8단계 기준)
6. 각 Phase 결과를 캐시하여 재실행 시 스킵 옵션 (파라미터 변경 시 무효화)
7. 파이프라인 설정 저장/불러오기 (JSON):
   - 모든 Phase의 파라미터
   - 가속화 조합 선택 상태
   - 완료된 Phase 목록

교육 모드 추가:
- 각 Phase 진입 시 알고리즘 설명 모달 표시 (토글)
- 핵심 수식 MathJax 렌더링
- "원본 C++ 코드 보기" 토글 (ch13 소스 코드 하이라이팅)
```

### 검증 기준
- [ ] 전체 실행 시 Phase 1~8 순차 완료
- [ ] 스텝 모드에서 각 Phase 독립 실행
- [ ] 파이프라인 설정 JSON 저장/불러오기

---

## Step 13 — 최종 통합 및 배포

### 프롬프트

```
다음 사항을 최종 점검하고 배포 환경을 구성해 줘.

1. vite.config.ts COOP/COEP 헤더 확인:
   server.headers:
     'Cross-Origin-Opener-Policy': 'same-origin'
     'Cross-Origin-Embedder-Policy': 'require-corp'

2. 빌드 최적화:
   - WASM 파일 gzip 사전 압축
   - Three.js tree-shaking (필요 모듈만 import)
   - 동적 import로 WASM 모듈 지연 로드

3. 에러 바운더리:
   - WASM 로드 실패 시 폴백 (JS 순수 구현 기본 제공)
   - WebGPU 미지원 시 WebGL 자동 폴백

4. README.md 작성:
   - 빌드 요구사항 (emsdk, Node.js, CMake)
   - KITTI 데이터셋 준비 방법
   - 합성 데이터로 테스트하는 방법
   - 각 Phase 교육 목적 설명

5. `npm run build` 후 `dist/` 확인:
   - WASM 파일 포함 여부
   - 번들 크기 리포트
```

### 검증 기준
- [ ] `npm run build` 성공 (오류 없음)
- [ ] `npx serve dist` 로컬 실행 시 전체 기능 동작
- [ ] KITTI 없이 합성 데이터로 Phase 1~8 완주 가능
- [ ] 퍼포먼스 대시보드에 모든 Phase 결과 기록

---

## 구현 순서 권장 (의존성 기준)

```
Step 0 (프로젝트 초기화)
  └─ Step 1 (WASM 빌드 환경)
       └─ Step 2 (로더/바인딩)
            ├─ Step 3 (Phase 1: 데이터셋) ─ Step 4 (Phase 2: 검출)
            │                                   └─ Step 5 (Phase 3: 스테레오 매칭)
            │                                        └─ Step 6 (Phase 4: 삼각화)
            │                                             └─ Step 7 (Phase 5: 추적)
            │                                                  └─ Step 8 (Phase 6: PnP)
            │                                                       └─ Step 9 (Phase 7: BA)
            │                                                            └─ Step 10 (Phase 8: 3D 뷰어)
            └─ Step 11 (퍼포먼스 대시보드) ─ 병렬 가능
Step 12 (파이프라인 통합) ─ Step 10 완료 후
Step 13 (배포) ─ Step 12 완료 후
```

---

## 핵심 기술 결정 사항

| 항목 | 선택 | 이유 |
|------|------|------|
| g2o 대체 | 경량 LM 자체 구현 | g2o WASM 빌드 복잡도 높음, 교육 목적엔 직접 구현이 학습에 유리 |
| OpenCV | opencv-wasm (npm) | 공식 사전 빌드 지원, LK/GFTT 포함 |
| 병렬화 | Emscripten pthreads | SharedArrayBuffer 기반, Web Worker 추상화 필요 없음 |
| 3D 렌더링 | Three.js (WebGL 기본) + Three.js WebGPU renderer | 동일 씬 그래프로 두 백엔드 전환 가능 |
| 상태 관리 | Zustand + immer | 파이프라인 단계별 독립 상태, 불변 업데이트 |

---

## 잠재적 난관 및 대응

| 난관 | 대응 |
|------|------|
| SharedArrayBuffer 제한 (COOP/COEP) | vite dev server에 헤더 설정 + 배포 시 서버 설정 가이드 제공 |
| KITTI 데이터 용량 큼 | 합성 체커보드 데이터 생성기 내장 + 소수 프레임(10장) 테스트 모드 |
| opencv-wasm LK 성능 | SIMD 빌드 플래그 적용 + 해상도 0.5× 다운샘플 유지 |
| WebGPU 브라우저 지원 | Chrome 113+/Edge 113+ 감지 후 폴백 처리 |
| g2o 미사용으로 최적화 품질 저하 | 교육 목적에 맞게 충분한 정밀도의 LM 구현 + 수렴 시각화로 보완 |
