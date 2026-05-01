# Step 01 — Dataset Loader (KITTI)

## 학습 목표

- [x] KITTI 스테레오 쌍과 calibration 파일이 어떻게 구성되는지 이해
- [x] `calib.txt`의 4개 projection matrix(P0~P3)에서 fx/fy/cx/cy/baseline 추출
- [x] downsample 비율을 적용했을 때 K 행렬이 어떻게 스케일되는지 관찰

## C++ 원본 매핑

- 파일: `ch13/include/myslam/dataset.h` + `ch13/src/dataset.cpp::Dataset::Init()`
- 핵심 함수: `Dataset::NextFrame()` — `image_{0,1}/{idx:06d}.png` pair → `Frame`
- 책 구현은 downsample을 0.5로 고정. 본 플레이그라운드는 0.25/0.5/1.0 토글로 학습 목적 가변.

## UI

- ParamPanel: start frame slider(0..4) · downsample toggle(0.25 / 0.5 / 1.0)
- Input View: 좌/우 회색조 PNG canvas (downsample 적용 후 해상도)
- Output View: P0~P3 카드 — fx, fy, cx, cy, t, K, baseline
- VerifyGate(자동 5건): calib parsed · baseline > 0 · frame loaded · L/R dims match · downsample 적용

## 알고리즘

- [x] KITTI calib parser (TS) — `parseKittiCalib(text, downsample)` in `src/lib/kitti.ts`
- [ ] (대체) WASM `stb_image` 디코딩 — 현재는 브라우저 native `ImageBitmap` 사용
- [x] 이미지 로드 — `loadKittiFrame(dir, idx, downsample)` (canvas 기반 회색조 변환)

## 가속 경로

- [ ] CPU scalar (해당 없음 — 단순 I/O 위주)
- [ ] CPU SIMD / MT / WebGL / WebGPU (해당 없음)

## 검증

- [x] 자동 수치 게이트 — VerifyGate 5건
- [x] 시각 수동 게이트 — 좌/우 disparity 관찰 + extrinsic t 변화
- [x] 원본 C++ 결과와의 diff — P1 baseline = 0.537151 m (실측 ≤ 1e-6)

## 학습 노트

`calib.txt`의 각 행은 3×4 projection matrix `P_i = K_i [R_i | t_i]`이지만 KITTI의 정류 스테레오에서는 모든 카메라가 같은 회전을 공유하므로 `R = I`로 간주하고 `t`만 따로 추출한다. baseline은 `||t||`로 계산하며 P0(좌측 회색조) 기준으로 P1은 -baseline_x만큼 떨어져 있다.

본 플레이그라운드는 KITTI EULA 회피와 결정론적 재현을 위해 **합성 mini fixture**(`scripts/gen-kitti-mini.ts`)를 사용한다 — Node 내장 zlib + 자체 CRC32로 5프레임 1226×370 회색조 PNG(`checkerboard + 앵커 마커`)를 생성. 실 KITTI 시퀀스가 준비되면 `public/datasets/kitti05-mini/`로 드롭인 교체 가능.

### 의도된 실패

- 손상된 `calib.txt` 입력 시 `parseKittiCalib`가 throw → React Query의 `error` 분기가 실행되어 카드에 에러 메시지 표시.
- downsample을 0.25로 줄이면 baseline은 그대로지만 fx/fy/cx/cy 모두 0.25배 — 학습자가 K 행렬 의존성을 즉시 관찰 가능.
