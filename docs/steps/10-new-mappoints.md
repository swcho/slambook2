# Step 10 — New MapPoints via Keyframe

## 학습 목표

- [x] 새 KF에서 미추적 영역을 어떻게 재검출 + 재삼각화하는지 이해
- [x] mask-existing(±10 box) vs full-redetect 토글의 효과 비교
- [x] carry-over set + new triangulation의 두 색상 시각화

## C++ 원본 매핑

- 파일: `ch13/src/frontend.cpp::Frontend::TriangulateNewPoints` + `Frontend::DetectFeatures`(mask 인자 사용)
- 핵심: 기존 KF의 features 좌표 주변에 ±10 사각 mask를 그리고 그 외 영역에서만 GFTT 재검출 → stereo LK → triangulation.

## UI

- ParamPanel:
  - redetect mode (mask-existing(ch13 default) / full-redetect)
  - mask radius slider (1..30 px)
  - new KF index(1..4) · num_features parameters · stereo LK params · depth band
- Input View: 새 KF의 좌측 + carry-over keypoints overlay (청록색)
- Output View:
  - **2D overlay** — carry-over(청록) + new(노랑) keypoints
  - **Scene3D** — 두 색상 클러스터의 3D 분포(r3f + drei)
  - **시퀀스 차트** — 4개 후보 KF 각각의 carry/new 누적 막대차트
- VerifyGate(자동 4건): 2 WASM · frame 0+i+calib · new ≥ 10 · depth ∈ [1, 80] m

## 알고리즘

- [x] 신규 C++ 없음 — features + triangulation WASM 재사용
- [x] mask-existing(±10 box, ch13 default)
- [x] full-redetect (학습용 비교)

## 가속 경로

- [ ] 가속 적용 없음 (mask 그리기 + 기존 WASM 호출)

## 검증

- [x] 자동 수치 게이트 — 4 KF 후보 모두 VerifyGate 통과
- [x] 시각 수동 게이트 — mask radius를 줄이면 노랑(new)이 청록(carry) 코너 위에 침범하는 모습 관찰
- [x] 원본 C++ 결과와의 diff — DetectFeatures의 mask 인자 동작 일치

## 학습 노트

새 KF에서 그대로 GFTT를 다시 돌리면 기존에 살아남은 features 좌표 위에 새 점이 또 만들어져 중복이 발생한다. ch13의 해결책은 **간단한 사각 mask(±10)** — 기존 features의 작은 영역을 차단해 거기서는 GFTT 응답이 0이 되도록 한다. 이는 GFTT의 `cv::Mat mask` 인자에 직접 넘기는 방식.

**carry-over set 시뮬레이션**: 본 Step은 영속 Map 매니저가 없으므로(Phase H Step 12에서 도입) frame 0의 stereo+temporal LK survival을 carry-over로 정의했다. 즉 frame 0에서 stereo triangulation으로 만든 3D 점을 frame 0 left → frame[i] left LK로 추적해 살아남은 점들. mini fixture는 합성 fixture라 frame 0과 frame[i]의 카메라 frame 차이가 거의 없어 이 시뮬레이션이 잘 동작한다 — 실 KITTI에서는 Step 8 PnP의 상대 pose로 carry-over 3D 좌표를 transform 해야 함(Step 13에서 합류).

**mask-existing vs full-redetect 비교**가 본 Step의 핵심 학습 포인트 — full-redetect는 carry-over 영역에서도 GFTT가 동작해 노랑 새 점이 청록 위에 겹친다. mask-existing은 그 겹침을 깔끔히 제거. 학습자가 mask radius를 30 → 2로 줄이면 그 효과가 점진적으로 사라지는 모습을 관찰할 수 있다.

### 의도된 실패

- mask radius = 0 → mask 효과 무효, mask-existing이 full-redetect와 동일 결과.
- depth band를 1~5 m로 좁히면 → 멀리 있는 candidate가 모두 rejected, new MapPoint 수가 임계 미달.
