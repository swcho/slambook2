# Step 06 — Initial Map Construction

## 학습 목표

- [x] 첫 KeyFrame을 world frame anchor로 설정하는 방법
- [x] stereo triangulation으로 초기 MapPoint cloud 생성
- [x] r3f + drei로 KeyFrame frustum + 3D 포인트 클라우드 시각화

## C++ 원본 매핑

- 파일: `ch13/src/frontend.cpp::Frontend::BuildInitMap`
- 핵심 흐름: `DetectFeatures()` → `FindFeaturesInRight()` → `triangulation` → `Map::InsertKeyFrame` + `InsertMapPoint`
- 첫 frame의 SE(3) pose는 identity(world = camera_left at frame 0).

## UI

- ParamPanel: maxFeatures · GFTT params · LK params · num_features_init threshold
- Input View: 좌/우 이미지 + GFTT/LK overlay
- Output View:
  - **KeyFrame 카드** — pose + feature 수
  - **Scene3D (r3f + drei)** — 좌/우 frustum (시안=좌, 자홍=우) + 초기 MapPoint cloud (밝기 ∝ depth)
- VerifyGate(자동 5건): WASM 로드 · L/R/calib · landmarks ≥ num_features_init(50) · all depth > 0 · depth ∈ [1, 80] m

## 알고리즘

- [x] 신규 C++ 없음 — Step 3/4/5의 features + triangulation WASM 재사용
- [x] r3f Scene3D 컴포넌트 — three.js 0.184 + @react-three/fiber@8 + drei@9

## 가속 경로

- [ ] 가속 적용 없음 (로직 조립 + 시각화)

## 검증

- [x] 자동 수치 게이트 — VerifyGate 5건 통과
- [x] 시각 수동 게이트 — 마우스 회전/줌으로 깊이 분포 확인, frustum positioning 자연스러움
- [x] 원본 C++ 결과와의 diff — Step 3/4/5 통과 + BuildInitMap 로직 그대로

## 학습 노트

ch13의 BuildInitMap은 약 30줄짜리 함수지만 SLAM의 "초기화" 본질을 압축한다 — feature 검출 → stereo 매칭 → triangulation → map 등록의 4단계가 첫 frame에서 일제히 수행되어 그 이후 모든 추적의 anchor가 된다.

본 Step의 시각화 가치는 **3D 깊이 분포를 직관적으로 보는 것**. KITTI 05 mini fixture(checkerboard 합성)에서는 깊이가 ~20 m에 cluster되어 frustum 앞에 평면 형태로 나타나지만, 실 KITTI 시퀀스에서는 가까운 차량(5~10 m), 도로(10~30 m), 빌딩(30~80 m)이 연속 분포로 보인다 — 본 PR의 mini fixture는 학습용 sanity, 실 시퀀스는 Phase I 후속 작업.

`@react-three/fiber@8`을 사용한 이유: peer dependency가 React 18(v9는 React 19 강제). drei@9도 동일. lazy chunk로 분리해 초기 번들 gzip은 104 KB 유지(Scene3D vendor chunk 245 KB는 Step 6 진입 시에만 로드).

### 의도된 실패

- num_features_init = 200으로 → KITTI mini는 features 200/200이라 OK지만 실 시퀀스에서는 텍스처 부족 frame에서 INIT 실패 → status가 INITING에 머무름.
- depth bound를 1~5 m로 줄여서 → 멀리 있는 점들이 모두 rejected, 초기 cloud가 sparse해지고 PnP 추적이 fragile해짐.
