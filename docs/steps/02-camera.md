# Step 02 — Camera Model

## 학습 목표

- [x] 핀홀 카메라의 forward(world→pixel) / inverse(pixel→world ray) 매핑 이해
- [x] 스테레오 baseline + extrinsic translation의 의미 파악
- [x] round-trip 오차로 좌표계 일관성 검증하는 방법 학습

## C++ 원본 매핑

- 파일: `ch13/include/myslam/camera.h` + `ch13/src/camera.cpp`
- 핵심: `Camera::pixel2camera`, `Camera::camera2pixel`, `Camera::pose()` (extrinsic SE3)
- 본 Step은 학습 범위를 **pure translation extrinsic**(KITTI 정류 stereo의 R=I 가정)으로 한정 — 회전 + 통합 SE(3)는 Step 7/8로 지연.

## UI

- ParamPanel: fx, fy, cx, cy 슬라이더 (KITTI 05 0.5× 기본) · 3D test point slider · P0/P1 extrinsic 선택
- Input View: 현재 점의 world coords + extrinsic 카드
- Output View: cameraToPixel/pixelToCamera 결과 + 60-point grid round-trip 오차 실시간 표시
- VerifyGate(자동 4건): wasm loaded · 60-point grid maxErr < 1e-5 · current-point round-trip · stereo extrinsic t 일치

## 알고리즘

- [x] 핀홀 forward/backward (원본) — `bind_camera.cpp` `cameraToPixel`/`pixelToCamera`
- [x] roundTripMaxError (자체) — 60-point 검증 함수
- (해당 없음 — 모델 자체 학습이 목적)

## 가속 경로

- [ ] 가속 적용 없음 (스칼라 산술 — 60 point round-trip < 1ms)

## 검증

- [x] 자동 수치 게이트 — round-trip maxErr 임계 1e-5 (실측 **1.06e-15**, machine precision)
- [x] 시각 수동 게이트 — extrinsic 토글 시 P0(0,0,0) → P1(-0.537,0,0)
- [x] 원본 C++ 결과와의 diff — KITTI 05 P1 baseline 일치

## 학습 노트

핀홀 모델은 가장 단순한 카메라 추상이지만 SLAM 전체에서 끊임없이 등장한다. `pixel = K · (R · world + t) / depth`의 양방향이 모두 자주 호출되므로 부동소수점 오차가 누적될 가능성을 처음부터 검증해 두는 것이 좋다.

본 Step의 round-trip 오차가 **1.06e-15**인 것은 single matrix multiply + division만 거치기 때문. Step 5(Triangulation)부터는 SVD/sparse solve가 들어가 오차가 1e-13~1e-12 단위로 커지며, Step 8(PnP)/Step 11(BA)는 LM 반복 수렴 + outlier rejection이 더해져 1e-3~1e-1 단위가 된다. 본 Step은 그 **최저 오차 floor**를 학습자가 직접 관찰하는 자리다.

WASM 바이너리는 18 KB — Eigen/Sophus 의존이 없어 매우 작다(Step 5의 28 KB Eigen-only triangulation, Step 8의 391 KB g2o-PnP와 비교).

### 의도된 실패

- fx 또는 fy를 0으로 설정 → division by zero, NaN 전파.
- cx를 이미지 중심에서 크게 벗어난 값으로 → 투영점이 화면 밖으로 나가 round-trip이 음의 depth를 만들 수 있음.
