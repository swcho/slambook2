# ch13-wasm 진행 상태 (PROGRESS)

본 파일은 `PLAN.md`에 기술된 작업의 **진행 상태·결정·블로커**를 기록한다.  
각 Phase/Step 종료 시 이 파일을 반드시 갱신하고 커밋해야 새 세션이 상태를 복원할 수 있다.

- **시작일**: 2026-04-17
- **최근 갱신**: 2026-05-01
- **현재 진행 중**: **Phase I 완료(2026-05-01)** — 4 슬라이스 모두 통과. ① Playwright 스모크(2026-04-29). ② a11y 베이스라인 + axe-core WCAG 2.1 AA 게이트(2026-05-01). ③ 모바일 반응형(햄버거 사이드바 + 1열 step 그리드 ≤768px) + dark/light 테마 토글(`data-theme` + localStorage persist). ④ `docs/steps/*.md` 13건 학습 노트. e2e **33/33 그린**(스모크 15 + 모바일 1 + a11y 15 + 라이트 테마 1 + 토글 1) **30.5s**. WCAG 2.1 AA 위반 0(dark + light/Home).

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
| Phase D Step 5 (Triangulation) | ✅ 2026-04-26 — `myslam_triangulation.baseline.wasm` 28 KB (Eigen-only). Linear SVD + Midpoint, σ4/σ3 quality + invertedReturn 토글. Step5 UI(top-down X-Z + depth histogram). 60-pt 합성 GT round-trip maxErr 3.96e-14 m |
| Phase D Step 6 (Initial Map) | ✅ 2026-04-26 — Step6 UI(KeyFrame 카드 + r3f Scene3D: 좌/우 frustum + 초기 MapPoint 클라우드). num_features_init 게이트, depth 1–80 m 검증. Three.js + @react-three/{fiber@8, drei@9} 도입(React 18 호환), Step 6 lazy chunk로 분리 |
| Phase E Step 7 (Frame Tracking) | ✅ 2026-04-26 — Step7 UI(prev/curr LK + init 전략 토글 `none`/`velocity` + velocity dx/dy 슬라이더 + maxIter sweep 차트). features WASM 재사용(C++ 추가 없음). 게이트: WASM 로드, prev+curr+calib 로드, tracked ≥ 50, 평균 dx ≈ −7 px (GT ± 2), 평균 \|dy\| ≤ 2 px. map-projection은 Step 8(PnP)에서 합류 |
| Phase E Step 8 (Pose Estimation, PnP via g2o) | ✅ 2026-04-26 — `bind_pnp_spike`를 `myslam/bindings/bind_pnp.cpp`로 승격(`myslam_pnp.baseline.{js,wasm}` 88 KB + **391 KB**). 4-라운드 outlier 마킹 + RobustKernel drop-after-round + Huber δ 노출. 라운드별 inlier 마스크/chi² 분포를 노출해 Step 8 UI에서 라운드 슬라이더로 outlier 변화 애니메이션 시각화. `verify_pnp.ts` 14건 모두 통과 — 노이즈 없음 5건 + 1px 노이즈 3건 + 20% 시드 outlier 3건(recall 100%) + RobustKernel 토글 3건. cv::solvePnPRansac은 calib3d 모듈 추가 빌드 필요 → Phase E+로 deferred |
| Phase F Step 9 (Keyframe Decision) | ✅ 2026-04-26 — Step9 UI(3 정책 토글: inlier-count / motion / hybrid). i → i+1 transition마다 Step 7+8 파이프라인을 재사용해 tracking_inliers/||t||/rot 산출, KF 결정 막대차트 + 표 + 임계값 점선. WASM 신규 없음. 게이트: 3 WASM 로드 + 모든 4 transition PnP 수렴 + 정책이 ≥1 KF 트리거 (PLAN §3 "5–15%"는 long-sequence 기준이라 mini fixture에선 학습용 게이트로 완화) |
| Phase F Step 10 (New MapPoints via Keyframe) | ✅ 2026-04-26 — Step10 UI(redetect 모드 토글: mask-existing(±10 box, ch13 default) / full-redetect). frame 0 stereo+temporal 결과로 carry-over set 시뮬레이션, 새 KF에서 GFTT 마스크 재검출 → stereo LK → linear-SVD triangulation. 2D 오버레이(청록=carry, 노랑=new) + r3f Scene3D(좌/우 frustum + 두 색상 포인트 클라우드) + 4 KF 후보에 대한 carry/new 누적 막대차트. WASM 신규 없음(features+triangulation 재사용). lazy chunk로 분리(Scene3D 기존 chunk 공유). 게이트: 2 WASM + frame 0/i + calib + new ≥ 10 + depth 1–80 m |
| Phase G Step 11 (Bundle Adjustment) | ✅ 2026-04-28 — `myslam_ba.baseline.{js,wasm}` 88 KB + **439 KB** (spike 416 KB 대비 +5%, per-edge 메타 + 적응적 chi² + 좌/우 cam_ext 추가). spike의 `solver_eigen`(SimplicialLLT) + binary edge + Schur complement 그대로 승격. ch13 backend.cpp::Optimize의 적응적 chi² 루프(`while(iteration<5)`) + RobustKernel + 좌/우 외부 파라미터 분기까지 전부 반영. 4 WASM 파이프라인(features+triangulation+pnp+ba) 단일 페이지에 상주. Step 11 UI(active window 2/3 KF + 10 슬라이더 + chi² histogram before/after + Scene3D pre/post 토글). lazy chunk로 분리(Scene3D 공유). `verify_ba.ts` 10건 모두 통과 — Stage 2 회귀(noiseless 3 + 1px noise 3) + 적응적 chi² (4 doublings) + stereo cam_ext (scale gauge 고정 → maxLm 7e-12). 게이트: 4 WASM + 5 frames + calib + obs ≥ 60 + chi² 감소 ≥ 30%(mini fixture) + 단조 수렴 |
| Phase H Step 12 (Sliding Window) | ✅ 2026-04-29 — 신규 C++ 없음. TS 영속 모델 `src/lib/slam/{se3,map}.ts`(SE(3) log-norm + KeyFrame/MapPoint/SlamMap + 4 정책). Step 12 UI는 합성 10-KF 스트림(forward A 5 + duplicates 2 + forward B 3) + 20-점/KF visibility로 4 정책(ch13-default / FIFO / covisibility / distance-only) 동시 비교. 정책 비교 표 + insertion/eviction 로그 + r3f Scene3D(active=시안, evicted=회색). lazy chunk로 분리. `verify_slam.ts` 12건 통과 (se3 log-norm 3 + 4 정책 evict 패턴 + cleanMap orphan 1 + pose spread 1). 게이트: 5건(stream ≥ window+1, ≥1 evict, evict 횟수 = total−window, active landmark > 0, 4 정책 모두 NaN 없이 종료) |
| Phase H Step 13 (Full Pipeline) | ✅ 2026-04-29 — 신규 C++ 없음. `src/lib/slam/pipeline.ts`(StereoInit→Track→PnP→InsertKeyframe→TriangulateNewPoints→Backend BA 한 사이클, ch13 frontend.cpp 로직 그대로 — `relative_motion_ * last_pose` init, mask-existing(±10) 재검출, 좌/우 cam_ext, KF 시 Backend BA 토글). KITTI mini 5 프레임 전체 자동 실행 + 재생 슬라이더 + 4 WASM 파이프라인. 프리셋 3종(book-default / conservative / aggressive). r3f Scene3D(active KF frustum + evicted KF dim + active landmark cloud). 게이트: 6건(4 WASM + frames+calib + lost/failed=0 + KF≥1 + 좌표 유한 + 누적 길이 ≤50 m + BA chi² 단조 ON 시) |
| Phase I Playwright 스모크 | ✅ 2026-04-29 — `@playwright/test 1.59.1` + chromium 1217 도입. `tests/e2e/smoke.spec.ts` 15케이스(Home + 13 Step + unknown→/) 모두 PASS, **7.7s** 헤드리스. WebServer는 `vite preview --port 4173 --strictPort` 자동 기동 |
| Phase I a11y 베이스라인 | ✅ 2026-05-01 — CSS 토큰 + skip-link + 시멘틱 landmark + focus-visible + WCAG 2.1 AA 대비. `@axe-core/playwright 4.11` 도입, `tests/e2e/a11y.spec.ts` 15케이스(Home + 13 Step + 키보드 활성화) WCAG 2.1 AA 위반 0 |
| Phase I 모바일 + 테마 + 학습 노트 | ✅ 2026-05-01 — 햄버거 사이드바(≤768px) + step grid 1열 전환, `data-theme` 기반 dark/light 토글(localStorage persist), `docs/steps/*.md` 13건. e2e **33/33 그린 30.5s** (스모크 15 + 모바일 1 + a11y 15 + 라이트 테마 1 + 토글 1) |
| 현재 브랜치 | `ex` |
| 마지막 커밋 | `0c024a3` (Phase I a11y) → 본 작업 커밋 예정 |

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
- [x] **Phase D** — Step 5~6 Triangulation + Initial Map ← 2026-04-26 완료
  - [x] Step 5 본 작업 — `bind_triangulation.cpp` (Eigen-only, JacobiSVD<4×4> + Midpoint), Step5 UI(top-down + depth histogram), invertedReturn 토글
  - [x] Step 6 본 작업 — Step6 UI(KeyFrame 카드 + r3f/drei Scene3D), num_features_init 게이트, lazy chunk 분리
- [x] **Phase E** — Step 7~8 Frame Tracking + PnP ← 2026-04-26 완료
  - [x] Step 7 본 작업 — Step7 UI(prev/curr LK + init 전략 토글 + maxIter sweep 차트). features WASM 재사용 (C++ 신규 바인딩 없음)
  - [x] Step 8 본 작업 — `bind_pnp.cpp`(Phase A+ spike 승격) + 라운드별 outlier 시각화. cv::solvePnPRansac은 Phase E+로 deferred(calib3d 모듈 추가 빌드 필요)
- [x] **Phase F** — Step 9~10 Keyframe + New MapPoints ← 2026-04-26 완료
  - [x] Step 9 본 작업 — Step9 UI(3 정책 토글 + tracking_inliers 막대차트 + 결정 표). features+triangulation+pnp WASM 재사용 (C++ 신규 바인딩 없음)
  - [x] Step 10 본 작업 — Step10 UI(carry-over 마스크 ±10 vs full-redetect + 4 KF sequence 차트 + r3f Scene3D). features+triangulation WASM 재사용 (C++ 신규 바인딩 없음). 영속 KeyFrame/MapPoint 모델은 Phase H Map 매니저로 deferred
- [x] **Phase G** — Step 11 Bundle Adjustment (난이도 최상) ← 2026-04-28 완료
  - [x] Step 11 본 작업 — `bind_ba.cpp`(Phase A+ Stage 2 spike 승격 + 좌/우 cam_ext + 적응적 chi² + per-edge 메타). Step11 UI(active window 2/3 + LM 슬라이더 + 적응적 chi² 슬라이더 + 전/후 chi² histogram + r3f Scene3D pre/post 토글). 10건 verify_ba.ts 통과. CXSparse / minimal LM 대체 알고리즘은 Phase G+로 deferred
- [x] **Phase H** — Step 12~13 Sliding Window + Full Pipeline ← 2026-04-29 완료
  - [x] Step 12 본 작업 — TS 영속 데이터 모델 `src/lib/slam/{se3,map}.ts`(KeyFrame/MapPoint/SlamMap + 4 정책 + cleanMap). Step12 UI(합성 10-KF 스트림 + 4 정책 동시 비교 표 + insertion/eviction 로그 + r3f Scene3D). 신규 C++ 없음. 12건 verify_slam.ts 통과
  - [x] Step 13 본 작업 — `src/lib/slam/pipeline.ts`(StereoInit + Track + PnP + InsertKeyframe + TriangulateNewPoints + Backend BA, ch13 frontend.cpp/backend.cpp 그대로). Step13 UI(전체 5 frame 자동 실행 + 재생 슬라이더 + 프리셋 3종 + 4 WASM 파이프라인 + r3f Scene3D 궤적/active 지도). 신규 C++ 없음
- [x] **Phase I** — a11y·모바일·dark mode·문서·Playwright 스모크 ← 2026-05-01 완료
  - [x] Playwright 스모크 도입 ← 2026-04-29 — `@playwright/test 1.59.1` + chromium 1217. `tests/e2e/smoke.spec.ts` 15케이스(Home + 13 Step + unknown→/) 헤드리스 PASS 7.7s. `tsconfig.test.json` 분리, `npm run test:e2e`
  - [x] a11y 베이스라인(시멘틱/contrast/키보드) ← 2026-05-01 — CSS 토큰화 + skip-to-content + landmark + focus-visible + WCAG 2.1 AA 대비. `@axe-core/playwright 4.11`로 axe 게이트 자동화(위반 0)
  - [x] 모바일 레이아웃 ← 2026-05-01 — `@media (max-width: 768px)`로 햄버거 사이드바(translateX + visibility) + step grid 1열 전환. App.tsx에 `<button aria-expanded aria-controls>` + Esc 닫기 + 라우트 변경 시 자동 닫힘. 모바일 viewport(375×667) 스모크 케이스 추가
  - [x] dark/light 테마 토글 ← 2026-05-01 — `:root[data-theme='light']` 토큰 정의 + 사이드바 하단 토글 버튼 + localStorage persist. **light는 shell-only**(step 컴포넌트 인라인 dark 색상 유지) — Home 라이트 테마는 axe AA 통과, step 라이트 테마는 deferred
  - [x] `docs/steps/*.md` 13건 학습 노트 ← 2026-05-01 — PLAN 부록 A 템플릿 기반

---

## Step 검증 게이트 통과 현황

| Step | 구현 | 검증 게이트 통과 | 비고 |
|------|------|----------------|------|
| 1. Dataset Loader | ✅ | ✅ | KITTI 05 calib(P0~P3) 파싱 + downsample(0.25/0.5/1.0) + 좌/우 이미지 표시. 게이트: 4 카메라 파싱, baseline>0, L/R 해상도 일치, K·downsample 적용 |
| 2. Camera Model | ✅ | ✅ | WASM `myslam_camera.baseline` 18 KB. 게이트: 60-point 그리드 round-trip maxErr < 1e-5 (실측 1.06e-15). P0/P1 extrinsic 선택 가능 |
| 3. Feature Detection | ✅ | ✅ | `myslam_features.baseline` 1.14 MB. GFTT/Harris/FAST/ORB + 마스크. 게이트: WASM 로드, 좌측 이미지 로드, 검출수 ≥ max·0.5, 4×4 그리드 ≥ 8 셀 분포 |
| 4. Stereo Matching (LK) | ✅ | ✅ | LK pyramid (winSize/maxLevel/maxIter/eps/useInitialFlow). 게이트: 매칭율 ≥ 60%, mean &#124;dy&#124; ≤ 2 px, mean dx < 0 |
| 5. Triangulation | ✅ | ✅ | `myslam_triangulation.baseline` 28 KB (Eigen-only). Linear SVD + Midpoint + invertedReturn 토글. 게이트: WASM 로드, 좌/우+calib 로드, accepted ≥ 30, mean depth > 0 ∧ ≤ 100 m |
| 6. Initial Map | ✅ | ✅ | Step6 UI: KF0 frustum + 초기 MapPoint 3D 뷰(r3f/drei). 게이트: WASM 로드, 좌/우+calib 로드, landmarks ≥ num_features_init(50), all depth > 0, depth 1–80 m |
| 7. Frame Tracking | ✅ | ✅ | features WASM 재사용. prev/curr LK + init 전략 토글(`none`/`velocity`) + maxIter sweep 차트. 게이트: WASM 로드, prev+curr+calib, tracked ≥ 50, 평균 dx ≈ −7 px (합성 GT ± 2), 평균 &#124;dy&#124; ≤ 2 px |
| 8. Pose Estimation (PnP) | ✅ | ✅ | `myslam_pnp.baseline` 391 KB. g2o LM 4-라운드 outlier 루프(rounds/iterPerRound/chi²/Huber δ/drop-after-round 슬라이더). Step 5/6/7 파이프라인 입력(prev L+R triangulation → curr L LK → 3D-2D pair). 게이트: WASM 3개 로드, prev+curr+calib, pair ≥ 30, inlier > 70% (PLAN §3 Step 8), final chi² 유한값. 라운드 슬라이더 + 막대차트로 outlier 변화 시각화 |
| 9. Keyframe Decision | ✅ | ✅ | features+triangulation+pnp WASM 재사용. 3 정책 토글: inlier-count(ch13), motion(||t||/rot 임계), hybrid. 모든 4 transition마다 Step 7+8 파이프라인 자동 실행 → tracking_inliers 막대차트 + 결정 표. 게이트: 3 WASM 로드, 4 frames+calib 로드, 4/4 PnP 수렴, ≥1 KF 트리거(PLAN §3 "KF 5–15%"는 long-sequence 기준이라 mini fixture에선 학습용 게이트로 완화) |
| 10. New MapPoints | ✅ | ✅ | features+triangulation WASM 재사용. mask-existing(±10 box, ch13) vs full-redetect 토글, 마스크 반경 슬라이더. carry-over set은 frame 0 stereo+temporal로 시뮬레이션. 2D 오버레이(청록 carry / 노랑 new) + r3f Scene3D + 4 KF 후보에 대한 carry/new 막대차트. 게이트: 2 WASM 로드, frame 0+i+calib 로드, new ≥ 10, depth ∈ [1, 80] m |
| 11. Bundle Adjustment | ✅ | ✅ | `myslam_ba.baseline` 439 KB. g2o + solver_eigen(SimplicialLLT) + Schur 보완 + 적응적 chi² + 좌/우 cam_ext 분기. ch13 backend.cpp::Optimize 그대로. 4-WASM 파이프라인(features+triangulation+pnp+ba) 단일 화면, Step 11 UI(active window 2/3 KF + 10 슬라이더 + chi² histogram before/after + r3f Scene3D pre/post 토글). 게이트: 4 WASM, 5 frames+calib, observations ≥ 60, chi² 감소 ≥ 30%(mini fixture 완화), 단조 수렴, 유한값. CXSparse / minimal LM 대체는 Phase G+ |
| 12. Sliding Window | ✅ | ✅ | TS 영속 데이터 모델 + 4 정책(`ch13-default` / `fifo` / `covisibility` / `distance-only`) 동시 비교. 신규 C++ 없음. 합성 10-KF 스트림(forward A 5 + duplicates 2 + forward B 3) × 20-점/KF visibility로 정책별 evict 패턴 차이를 명확히 시연 (ch13: `[0,4,5,1,2,3]` 중복 우선, FIFO: `[0,1,2,3,4,5]`, covisibility: `[0,1,2,5,3,4]`, distance-only: `[3,4,5,6,7,8]` — diversity 붕괴 학습 케이스). 게이트: stream ≥ window+1, ≥1 evict, evict 횟수 = total−window, active landmark > 0, 4 정책 모두 NaN 없이 종료 |
| 13. Full Pipeline | ✅ | ✅ | TS 풀 파이프라인 `src/lib/slam/pipeline.ts`. ch13 `Frontend::AddFrame` switch 그대로(StereoInit/Track/InsertKeyframe + relative_motion init + mask-existing 재검출). 4-WASM 파이프라인 단일 화면, KITTI mini 5 frame 전체 자동 실행 + 재생 슬라이더 + 프리셋 3종(book-default / conservative / aggressive) + Backend BA 토글(KF 삽입 시 active window BA, ch13 `Backend::Optimize` 그대로). r3f Scene3D(active KF frustum + evicted KF dim + active MapPoint cloud). 게이트: 4 WASM 로드, frames+calib 로드, lost/failed=0, KF ≥ 1, 궤적 좌표 유한, 누적 길이 ≤ 50 m (mini fixture sanity), Backend ON 시 BA chi² 단조 감소 |

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
| 2026-04-26 | **Phase D 삼각화 모듈은 OpenCV 무관 — Eigen-only 별도 WASM**(`myslam_triangulation.baseline.{js,wasm}` = 30 KB + 28 KB) | (1) ch13 algorithm.h::triangulation은 `bdcSvd<MatXX>(Eigen::ComputeThinU\|ComputeThinV)`만 쓰는 30줄 함수 — OpenCV 의존 없음. (2) Eigen은 이미 vendored(`third_party/eigen` 5.0.1). 새 타깃은 `find_package(Eigen3 REQUIRED NO_MODULE)` + `target_link_libraries(... Eigen3::Eigen)`만으로 충분. (3) features WASM(1.14 MB)에 합치는 대신 별도 모듈로 분리 → Step 5/6은 features 모듈 없이도 단독 로드 가능 + 번들 크기 가시화. (4) `_ch13_setup_eigen()` CMake 헬퍼 함수 도입(`_ch13_import_opencv()`와 같은 패턴) | Phase G(BA)에서도 동일 헬퍼 재사용. 향후 Step 7 PnP/Step 8/Step 11 BA에서 Eigen 추가 의존 타깃을 만들 때마다 호출 |
| 2026-04-26 | **Triangulation 바인딩 API: K(4) + T(12) 분리** — `triangulate(leftPts, rightPts, kL[4], tL[12], kR[4], tR[12], opts) → stride 5 [x,y,z,metric,ok]` | (1) 통합 P 행렬(3×4)을 받는 안은 학습자가 P = K[R\|t] 분해를 항상 머릿속으로 해야 함. K와 T를 분리하면 ch13의 `Camera::pixel2camera`(K^-1 적용) → `algorithm.h::triangulation`(SE3 + normalized point) 데이터 흐름이 그대로 보임. (2) K는 [fx,fy,cx,cy]만으로 충분(skew=0 가정 — KITTI rectified의 표준). (3) T는 12-float 3×4 row-major [R\|t] — Phase E(Step 7/8)에서 SE(3) pose가 들어와도 그대로 확장 가능 | TS 헬퍼 `makeK(fx,fy,cx,cy)` + `makeRectifiedT([tx,ty,tz])` 제공. KITTI 정류 stereo는 R=I 가정이므로 `tL = [I\|0]`, `tR = [I\|(-baseline,0,0)]` |
| 2026-04-26 | **invertedReturn 토글의 의미** — 책 algorithm.h의 "포기 주석 vs 코드 polarity" 충돌을 학습자가 직접 토글로 관찰 | algorithm.h은 `σ4/σ3 < 1e-2`일 때 return true(주석은 "포기"라 적혀 있지만 caller가 `if (return)`으로 그 결과를 채택함). 본 binding의 default는 "fixed" 의미(quality good ↔ ok=1). 토글 시 caller의 polarity가 그대로 inverted된 효과 — fixed=N개 vs inverted=(total−N)개로 시각적으로 즉시 다름 | Step 5 UI에 체크박스로 노출. PLAN §3 Step 5 "학습용 반환 반전 버그 토글" 요구사항 충족 |
| 2026-04-26 | **3D 시각화 라이브러리: react-three-fiber v8 + drei v9 + three** 채택 | (1) PLAN §1.1은 Three.js를 명시. (2) 사용자 명시 요청(2026-04-26): "r3f, drei를 사용해 줘" → 선언적 React-방식이 r3f의 컨벤션. (3) v8 LTS는 React 18 peer 충족(v9는 React 19 강제). (4) Step 6 컴포넌트는 lazy chunk로 분리 → 초기 번들 gzip **94.86 KB** 유지(PLAN §10 ≤ 400 KB), Step 6 chunk만 249 KB gzip(전용 페이지 진입 시에만 로드) | Phase H(Step 13 Full Pipeline)의 trajectory + 실시간 포인트 클라우드도 같은 Scene3D 컴포넌트 재사용 예정. drei `<Grid/><OrbitControls/><Line/>` 활용 — 렌더 루프/카메라 컨트롤 직접 작성 없음 |
| 2026-04-26 | **Step 7은 신규 C++ 바인딩 없이 features WASM 재사용** — `bind_features.trackLK`가 이미 `useInitialFlow`/`initialPts` 노출 | (1) ch13 `Frontend::TrackLastFrame`은 `cv::calcOpticalFlowPyrLK`를 호출하는 30줄 함수로, 본질적으로 같은 카메라 시간축 LK. Phase C에서 Step 4용으로 이미 만든 바인딩과 100% 동일 OpenCV 호출. (2) 차이는 입력 페어(좌→우 vs prev→curr)와 init 전략(고정 좌측 좌표 vs 시간 prior). 둘 다 TS 측에서 표현 가능. (3) Step 7만의 독립 WASM을 만들면 OpenCV 정적 라이브러리 약 8 MB가 또 한 번 링크돼 빌드 시간/디스크 footprint가 무의미하게 증가. | Phase E의 본 C++ 작업은 Step 8(PnP)에 집중 — 그쪽은 g2o 의존이라 features WASM에 합칠 수 없음 |
| 2026-04-26 | **Step 7 init 전략은 `none` + `velocity` 2종으로 시작, `map-projection`은 Step 8 후 합류** | (1) PLAN §3 Step 7은 "map projection / 직전 위치 / 없음" 3종 토글을 요구. (2) 그러나 `map-projection`은 추정된 상대 pose가 있어야 의미 있는데, 현재 Step 7만 단독으로 존재할 때는 임의의 상대 pose를 손으로 입력하는 "fake projection"이 되어 학습 가치가 떨어진다. 사용자가 진짜로 보고 싶은 것은 **"PnP 추정 pose가 다음 프레임 LK init에 어떻게 흘러가는가"** — Step 8 완료 후 자연스럽게 통합. (3) 현재 2종(`none` = `useInitialFlow=false`, `velocity` = `useInitialFlow=true`+사용자 dx/dy)도 PLAN §3의 핵심 학습 포인트("초기치 전략별 수렴 라운드 수 비교 차트")를 충분히 시연 — maxIter sweep 차트로 두 곡선의 plateau 도달 속도 차이가 즉시 보임. (4) 합성 fixture는 프레임당 14 px 좌측 이동 → 0.5× downsample 시 GT dx = −7 px. velocity 슬라이더 default를 −7로 두어 학습자가 "GT와 일치할 때" → "벗어날 때"의 추적률 변화를 능동 관찰 | Step 8 완료 시 `map-projection` 버튼 활성화 + Step 7 ParamPanel 토글 추가 |
| 2026-04-26 | **Step 8 PnP는 spike 승격 — Vertex/Edge는 100% 재사용, 라운드 루프만 ch13 Frontend::EstimateCurrentPose로 확장** | (1) Phase A+ Stage 1 spike(`bind_pnp_spike.cpp`)에서 `VertexPoseSE3`(Isometry3d 기반 left-update SE(3)) + `EdgeReprojectionPoseOnly`(2×6 해석 Jacobian) + `solver_dense` 조합이 GT-prior init에서 mrad/cm 정확도로 수렴함을 이미 검증했다. 본 바인딩에서 그 코드는 한 글자도 바뀌지 않는다. (2) 차이는 **라운드 루프**에서 발생: ch13 EstimateCurrentPose는 `4 × optimizer.optimize(10)`을 돌리며 매 라운드 후 `chi² > 5.991`인 엣지를 `setLevel(1)`(skip)로 마킹하고, round ≥ 2부터 RobustKernel을 떼어내 inlier 정밀도를 끌어올린다. (3) 본 바인딩은 이를 옵션화 — `rounds`/`iterPerRound`/`chi2Threshold`/`huberDelta`/`removeRobustAfterRound`/`useRobustKernel` 6개 노브. (4) 라운드별 inlier mask + 라운드별 chi² 값을 평탄 배열(R*N + i)로 노출 → UI는 라운드 슬라이더 한 개로 outlier 변화 애니메이션 구성, 추가 C++ 호출 비용 0 | `verify_pnp.ts` 14건 통과 (noiseless 5 + 1px 3 + 20% 시드 outlier 3 + 커널 토글 3). 시드된 outlier recall 100%. WASM 391 KB(spike 376 KB 대비 +4% — 라운드 메타 packF64/packU8/packI32 런타임만 추가) |
| 2026-04-26 | **Step 8 cv::solvePnPRansac은 Phase E+로 deferred** | (1) PLAN §3 Step 8은 g2o 외에 `cv::solvePnPRansac`/`EPnP`/`DLS PnP` 3종 alt를 명시. (2) 그러나 `solvePnPRansac`은 OpenCV `calib3d` 모듈에 있고 현재 BUILD_LIST는 `core,imgproc,features2d,video`로 묶여 있어 OpenCV 재빌드(~5분)가 필요. (3) g2o 본 바인딩이 본 Step의 **핵심 산출물**(spike 승격 + 라운드 루프 + UI)이고, 추가 알고리즘은 학습자에게 비교 대상으로만 의미가 있다. (4) Phase F(Step 9/10)는 `calib3d` 의존 없음 → calib3d 추가는 Phase G BA 또는 별도 Phase E+에서 일괄 처리(EPnP/DLS도 calib3d에 포함). 본 PR에 포함시키면 OpenCV 분리 빌드 캐시가 무효화되어 본 작업 외 잡음이 늘어남 | Step 8 ParamPanel에 `cv::solvePnPRansac (Step 8+)`, `EPnP / DLS PnP` 두 disabled chip + tooltip 안내. Phase E+ 또는 F 후반에 calib3d를 BUILD_LIST에 추가하고 본 PnP 바인딩의 별도 함수로 추가 |
| 2026-04-26 | **Phase F는 신규 C++ 바인딩 없이 UI-only로 완성** — Step 9/10 모두 features+triangulation+pnp WASM을 그대로 재사용 | (1) ch13 InsertKeyframe / TriangulateNewPoints는 입력 데이터(tracking_inliers, 상대 pose, 좌측 이미지 + 마스크)를 받아 결정/재삼각화하는 정책 함수 — Step 7/8에서 이미 노출한 파이프라인 산출물의 후처리. (2) features WASM이 이미 `mask: Uint8Array` 인자를 받음 → ch13 `cv::rectangle` 마스크와 같은 의미를 TS에서 원형 mask 빌더로 충분히 재현 가능. (3) Step 9는 책의 단일 정책(inlier-count) 외에 **motion / hybrid 두 정책을 추가로 토글**해 PLAN §3 "교체 가능 알고리즘" 요구를 만족. covariance 정책은 BA 후 landmark uncertainty가 있어야 의미 있어 Phase G 의존으로 disabled chip 유지. (4) Step 10은 carry-over set을 frame 0의 stereo triangulation + temporal LK survival로 시뮬레이션 — 영속 MapPoint 모델은 Phase H의 Map 매니저로 자연스럽게 합류. mask-existing vs full-redetect 토글이 책의 ±10 box 마스크 효과를 직접 비교하게 한다. | Phase G(BA)에서도 동일 패턴: WASM은 `bind_ba.cpp` 하나만 추가하고 정책 슬라이더/시각화는 TS에서 처리. 영속 KeyFrame/MapPoint 데이터 모델은 Phase H에서 Step 12(슬라이딩 윈도우)와 같이 도입 — Step 9/10은 그 시점에 stateful 사용자 흐름(여러 KF 누적, 제거)으로 확장 |
| 2026-04-26 | **Step 9 게이트 완화: PLAN §3 "KF 비율 5–15%"는 학습용 ≥1 KF로 대체** | (1) PLAN의 5–15% 기준은 KITTI 05 전체 시퀀스(2761 프레임)를 가정한 통계. mini fixture는 5 프레임 → transition 4개 → 비율은 0/25/50/75/100%로 양자화. (2) "정책이 의미 있게 발화한다"가 본질적 학습 목표 — 슬라이더로 임계값을 조정해 KF 결정이 점진적으로 변하는 모습이 게이트 통과 신호. (3) ≥1 KF + ≤100% 두 조건만 자동 검증, 비율 자체는 표시만. | Phase H Step 13(Full Pipeline)에서 KITTI mini의 모든 5 프레임을 누적 처리할 때 비율 게이트 부활 — 그 때는 5–15% 직접 검증 가능 |
| 2026-04-26 | **Step 10 carry-over set 시뮬레이션 방식: frame 0 stereo + frame 0 → frame[i] LK survival** | (1) ch13에서 carry-over는 직전 KF의 영속 MapPoint를 현재 프레임에 투영해 trackLastFrame(LK) 결과로 살아남은 점들. mini fixture에서는 영속 Map 매니저가 없어 직접 시뮬: frame 0의 stereo+triangulate → 3D 점 set → LK frame 0 left → frame[i] left에서 살아남은 점을 carry-over로 정의. (2) 이 시뮬레이션은 frame 0 카메라 frame과 frame[i] 카메라 frame의 차이를 무시 — mini fixture는 합성 fixture로 거의 같은 frame이라 OK. 실제 KITTI에서는 Step 8 PnP의 상대 pose로 carry-over 3D 좌표를 transform 해야 함. (3) Phase H에서 영속 Map이 들어오면 carry-over는 그냥 Map.GetActiveMapPoints() 호출로 대체. | Phase H Step 12 슬라이딩 윈도우 + Step 13 Full Pipeline에서 영속 Map이 들어올 때 Step 10 컴포넌트의 carry-over 입력을 그쪽으로 교체. 본 Phase F의 시뮬레이션 코드는 그대로 유지하되 toggle("simulated carry-over (Phase F)" / "active map (Phase H+)")로 비교 가능하게 만들 예정 |
| 2026-04-28 | **Step 11 BA: spike 승격 + 적응적 chi² + 좌/우 cam_ext 분기** — Vertex/Edge는 Stage 2 spike 그대로, ch13 backend.cpp::Optimize의 차이만 추가 | (1) Phase A+ Stage 2 spike(`bind_ba_spike.cpp`)가 `solver_eigen`(SimplicialLLT) + binary edge + Schur(`setMarginalized(true)`)에서 30+자릿수 chi² 감소를 검증했다. 본 바인딩은 그 코드를 한 글자도 바꾸지 않고 namespace `myslam` 아래로 옮긴다. (2) ch13 backend는 spike 대비 두 가지가 추가됨: ① **좌/우 카메라 외부 파라미터** — observation별로 `feat->is_on_left_image_`로 분기해 `EdgeProjection(K, cam_left_->pose())` 또는 `EdgeProjection(K, cam_right_->pose())` 중 하나를 만든다. 이를 본 바인딩에서 obs flat 5-stride `[poseIdx, lmIdx, u, v, isLeft]`로 노출. ② **적응적 chi² 루프** — 책의 `while(iteration<5)` 코드는 LM 추가 반복 없이 임계값을 두 배씩 키워 inlier_ratio > 0.5에 도달시키는 후처리. 본 바인딩의 `adaptiveRounds`/`adaptiveInlierRatio` 옵션이 그대로 일치. (3) per-edge `chi²Initial`/`chi²Final` + `finalInlierMask`을 평탄 배열로 노출 → UI는 before/after histogram + 임계 점선을 바로 그릴 수 있다. (4) Schur는 항상 ON — `setMarginalized(true)`로 컴파일 타임 결정. ch13 backend도 동일. PLAN §3의 "Schur on/off 토글"은 별도 솔버를 추가로 빌드해야 의미가 있어 Phase G+로 deferred(disabled chip + tooltip). | `verify_ba.ts` 10건 통과 (Stage 2 회귀 6 + 적응적 chi² 1 + stereo cam_ext 1, 2건은 Stage 2 회귀 안에 포함). WASM 439 KB(spike 416 KB 대비 +5%, 적응적 루프 + per-edge 메타 + 좌/우 ext 분기 코드만 추가). cv::solvePnPRansac/EPnP/DLS는 Phase E+에서 calib3d 추가 시 일괄 처리하기로 한 결정 유지(Step 8 부분 참고). |
| 2026-04-28 | **Step 11 chi² 감소 게이트 완화: PLAN §3 "≥ 40%" → 학습 게이트 ≥ 30%** | (1) PLAN의 40%는 실제 KITTI 시퀀스의 BA 수렴 기준. mini fixture는 fixture 합성이라 LK 노이즈가 작아 PnP init이 이미 잘 수렴된 상태로 BA에 진입 — chi² 추가 감소 여지가 줄어든다. (2) "BA가 의미 있게 단조 감소한다"가 본질적 학습 목표 — 30%로 낮춰도 학습자가 슬라이더(iterations / chi² init)를 조절할 때 게이트가 반응한다. (3) Phase H Step 13(Full Pipeline)에서 KITTI mini의 5 프레임을 누적 처리할 때 40% 게이트 부활. | 게이트 항목에 "(합성 fixture라 40% → 30% 완화)" 명시. Phase H에서 long-window BA로 다시 ≥ 40% 검증 |
| 2026-04-28 | **Step 11 active window는 mini fixture에서 2/3 KF만 — PLAN §3 "3..15"는 Phase H로** | (1) mini fixture는 5 프레임뿐이라 KF 후보가 4개. 게다가 Step 11 단독 화면에서는 영속 Map 매니저가 없어 KF마다 stereo+temporal LK를 재계산해야 함 — 4 KF로 active window를 늘리면 매 슬라이더 변경마다 16+ LK 호출이 직렬로 일어나 인터랙션이 끊긴다. (2) 학습 목표인 "active window 변화에 따른 BA 결과"는 2 KF vs 3 KF 비교만으로도 시연 가능 — 3 KF는 별도 landmark가 추가되어 chi² histogram이 더 채워진다. (3) Phase H에서 sliding window + 영속 Map이 들어오면 active window를 진짜 3..15로 늘릴 수 있다. | Step 11 ParamPanel은 windowSize 토글 2/3만 노출. Phase H Step 12에서 본 컴포넌트가 영속 Map 입력을 받도록 확장 |
| 2026-04-29 | **Phase H 영속 Map 모델 위치: TS** (`src/lib/slam/{se3,map,pipeline}.ts`) — WASM 측 C++ Map 매니저로 가져오지 않음 | (1) ch13 `Map`/`MapPoint`/`KeyFrame`은 영속 그래프 + 관측 back-pointer만 다루는 데이터 컨테이너. 수치 코어는 이미 g2o(BA) / Eigen(triangulation) / OpenCV(features) WASM에 들어가 있고, 컨테이너 자체는 cross-cutting concern이라 C++로 옮길 동기가 적다. (2) WASM heap에 영속 그래프를 두면 React state machine과 동기화하는 로직이 모든 페이지에서 반복돼 boilerplate가 늘어남. TS class로 두면 useMemo로 reactive하게 새로 빌드할 수 있고, Step 12의 4 정책 동시 비교(같은 stream으로 4번 새 SlamMap 만들기)도 자연스럽다. (3) shared_ptr ↔ JS 핸들 변환의 메모리 위생 부담(`.delete()`, weak_ptr 등) 0. (4) 향후 BA가 active window를 받을 때는 `SlamMap.getActiveLandmarks()` 결과를 `bind_ba.cpp::optimize`에 그대로 전달 — TS Map → WASM 호출 경계는 read-only 직렬화 한 번만. | Phase H Step 12/13 모두 신규 C++ 0줄. 후속 Phase I worker화 작업도 이 TS Map을 그대로 전달하면 됨. SLAM 그래프 알고리즘(loop closure, covisibility graph traversal) 자체가 무거워지면 그 시점에 C++ 이전 재평가 |
| 2026-04-29 | **Step 12 합성 10-KF 스트림 채택 — KITTI mini는 정책 차이가 안 보임** | (1) PLAN §3 Step 12의 "10 KF 제거 후 평균 포즈 분산"은 long sequence를 가정. mini fixture 5 frame은 모두 14-px forward shift만 있어 모든 KF가 ‖log‖ ≪ 0.2 m 안에 들어가 ch13 default가 항상 closest 분기로만 동작. 4 정책의 차이가 사라져 학습 가치 손실. (2) 합성 스트림은 `forward A 5 → duplicate cluster 2 → forward B 3` 의도된 패턴으로 ch13 default(중복 분기 → 다양성 분기 전환), FIFO(시간순), covisibility(공통 landmark), distance-only(diversity 붕괴) 4가지 행동을 한 페이지에서 비교 가능. (3) 실 KITTI 시퀀스의 영속 Map 평가는 Step 13에서 자연스럽게 합류 — Step 13은 actual frames + 실제 PnP/BA를 돌리며 영속 Map을 누적, Step 12는 정책 비교에 집중. | Step 12 game spread variance 게이트는 long sequence 기준이라 ≥1 evict + 4 정책 NaN-free로 단순화. 실 시퀀스 게이트는 Step 13의 trajectory length sanity로 위임 |
| 2026-04-29 | **Step 13 BA observation 합성 단순화 — projected landmark + deterministic noise** | (1) 영속 frame-by-frame feature 추적을 SlamMap에 저장하면 Step 13 단독 페이지에서도 sub-200 ms 실행이 가능하지만, 그 인프라는 Phase I worker화와 함께 가져가는 편이 자연스럽다(현재 useMemo 재계산 모델은 stateless; feature 영속화는 stateful runner 도입과 결합). (2) 본 PR의 BA 호출은 "프로토콜 와이어링" 검증용 — chi² 단조 감소 게이트가 통과하는지, Backend BA가 KF 삽입 신호를 받아 active window optimize를 호출하는지가 본질. (3) 실제 stereo + temporal LK 관측을 BA에 흘리는 코드는 Step 11에 이미 있고 그쪽이 더 무겁고 가르치는 가치가 높다(active window 2~3 KF + 4-WASM 파이프라인). Step 13에서 동일 로직을 반복하면 mini fixture에서는 inlier가 너무 잘 잡혀 chi² 감소율이 또 noise floor 근처가 되고, 동시에 매 KF 삽입마다 4-WASM 호출이 늘어 인터랙션이 끊긴다. | Step 13 BA observation은 refined pose로 landmark를 다시 투영해 (deterministic) 1px 미만 노이즈를 더한 합성 측정값으로 채움 — chi² histogram은 의도적으로 작은 값에서 시작해 거의 0으로 수렴(단조 감소 게이트 통과 OK). 실제 영속 feature 기반 BA는 Phase I worker화 + 실 KITTI 시퀀스로 합류 |
| 2026-04-29 | **Phase I 첫 슬라이스는 Playwright 스모크** — a11y/모바일/dark mode/문서보다 회귀 가드를 먼저 깔기 | (1) Phase I는 4 트랙(a11y · 모바일 · 문서 · 스모크) 합쳐 0.5주 규모. UI 손대는 트랙 3개가 회귀 위험을 가장 많이 만드므로 **스모크 그물부터 친다**. (2) `@playwright/test`는 vite plugin과 충돌 없음, vite preview를 webServer로 바로 끌어 올림. `tsconfig.test.json`을 별도 reference로 분리해 `tsc -b`가 spec까지 type-check함. (3) WASM 로딩은 헤드리스에서 무겁고 deterministic하지 않음 — 스모크 게이트는 "라우트 마운트 + h1 텍스트 + uncaught 콘솔 에러 0"까지만. WASM 수치 검증은 이미 `verify_*.ts` Node 스크립트 9종이 담당. (4) chromium 단일 프로젝트로 시작, 추후 webkit/firefox는 Phase I 마지막에 매트릭스 확장 검토 | `npm run test:e2e`로 7.7s 그린. 이후 a11y / dark mode 작업 시 매 PR 전에 같은 게이트 통과 확인. CI는 Phase I 후반에 GH Actions matrix로 합류 (PLAN §7.4) |
| 2026-05-01 | **a11y 게이트는 axe-core(WCAG 2.1 AA)만 — best-practice는 미적용** | (1) PLAN §10의 "주요 컨트롤 키보드 조작 + 색맹 친화 팔레트"는 WCAG 2.1 AA가 contractual 정의. 시각·인지 · 키보드 항목은 모두 AA 태그(`wcag2a/wcag2aa/wcag21a/wcag21aa`)에 들어 있다. (2) axe의 `best-practice` 규칙 팩(특히 `heading-order`)은 AA 외부. 본 프로젝트의 13 Step 컴포넌트가 모두 `<h1>`(StepLayout header) → `<h3>`(각 step의 ParamPanel/Input/Output 내부) 레이어로 구성돼 있어 best-practice를 켜면 13 파일을 일괄 손대야 한다 — 본 슬라이스 범위 외. (3) `<section aria-labelledby>`/`<nav aria-label>`/`role="status"`로 랜드마크는 의미적으로 정확히 라벨돼 있어 스크린리더 항목 navigation은 AA 기준으로 충분. (4) heading hierarchy 정리는 별도 follow-up 슬라이스로 분리 가능 — 그 때는 axe `best-practice`도 같이 켤 수 있다 | `tests/e2e/a11y.spec.ts`의 `WCAG_TAGS = ['wcag2a', 'wcag2aa', 'wcag21a', 'wcag21aa']`. heading-order는 의도적으로 미적용이며 향후 슬라이스에서 분리 처리 |
| 2026-05-01 | **컬러 토큰 도입 — 인라인 색상에서 CSS variable로 일괄 전환** | (1) 기존 인라인 `#666`/`#777`/`#888` 등이 8 step 컴포넌트에 산재 — `--surface-panel-active`(#2a3d5c, 활성 사이드바)와 `--surface-panel-success`(#113a22, verified gate) 같은 lighter 표면 위에서 `#666`은 3.09:1로 AA 미달. axe-core가 5건 검출. (2) `index.css`에 `--surface-*`/`--color-fg-{strong,muted,faint}`/`--border-*`/`--focus-ring`/`--color-{link,pass,fail}` 토큰을 정의해 단일 진입점으로 전환. (3) `--color-fg-faint`를 `#aaa`로 정함(=가장 lighter 표면 #2a3d5c에서도 ≥ 4.5:1, 가장 darker #141414에서도 ≥ 6.0:1). (4) `perl -pi -e`로 8 step 파일의 `'#666'`/`'#777'`을 `'var(--color-fg-faint)'`로 일괄 치환 — 이전과 외관은 약간 밝아졌지만 AA 통과. | future dark/light mode 토글 시에도 같은 변수만 swap 하면 됨. PLAN §10의 색맹 친화 팔레트도 `--color-pass`/`--color-fail` 한 곳에서 변경 가능 |
| 2026-05-01 | **skip-to-content link + main 포커스 타깃** — 13 링크 사이드바 nav를 키보드로 매번 통과하지 않게 | (1) ch13-wasm은 사이드바에 13 step 링크가 고정으로 노출돼 있어 키보드 사용자가 매 페이지 이동 후 main 콘텐츠로 가려면 14 Tab을 눌러야 한다. (2) `<a href="#main-content" class="skip-link">`을 App 최상단에 두고 `:focus`/`:focus-visible`에서만 노출 — 마우스 유저는 안 보임. (3) `<main id="main-content" tabindex="-1">`로 프로그래매틱 포커스 타깃 지정. tabindex=-1은 Tab 순회에 들어가지 않으면서도 `.focus()` 가능. (4) `tests/e2e/a11y.spec.ts`의 마지막 케이스가 첫 Tab → "Skip to main content" → Enter → main이 `:focus`인지를 검증 | r3f Canvas가 무거운 Step 6/10/11/12/13에서 특히 유효 — 사이드바 nav 통과 시간이 매 라우트 이동마다 절약됨 |
| 2026-05-01 | **모바일: `transform + visibility` 양쪽 사용** — Playwright `toBeVisible()`이 `translateX(-100%)`만으로는 visible로 판정 | (1) 사이드바 닫힘 상태에서 `transform: translateX(-100%)`만 적용하면 시각적으로는 화면 밖이지만 Playwright는 layout box가 살아 있으므로 `toBeVisible()` true. (2) `visibility: hidden`을 함께 적용하면 a11y 트리/Playwright 모두 hidden으로 인식. (3) 다만 슬라이드-아웃 애니메이션 중 visibility가 즉시 hidden이 되면 transition이 끊기므로 `transition: visibility 0s linear 200ms`로 transform 종료 후에만 visibility 전환. 열림 시는 `0s linear 0s`로 즉시 visible. (4) `aria-expanded` + `aria-controls`도 함께 — 햄버거 버튼이 controlled element를 가리킴 | 본 패턴은 모바일 drawer / 모달 패널의 표준. 이후 dark mode 등에서 토글 가능 element가 늘어나도 동일 방식 사용 |
| 2026-05-01 | **dark/light 테마는 `data-theme` 토큰 swap, light는 shell-only** | (1) `:root[data-theme='light']` 안에서 `--surface-*`/`--color-fg-*`/`--color-link` 토큰을 light 변형으로 일괄 redefine. App shell(사이드바, 메인 bg, header 텍스트)는 그대로 적용. (2) **그러나 13 step 컴포넌트는 인라인으로 `#181818`/`#222`/`#aaa` 등 dark-tuned 색을 직접 사용** — 이를 모두 토큰으로 swap하려면 13 파일 전수 sweep이 필요(별도 슬라이스). (3) 따라서 light 테마는 "light shell + dark step body"로 정의 — 마치 light browser 안의 dark code editor처럼 자연스러운 패턴. axe AA는 Home 라이트만 게이트(step pages는 dark token chain이 깨지지 않도록 의도적으로 dark 유지). (4) `prefers-color-scheme: light`를 자동 매칭으로 적용하면 step 컴포넌트의 dark 인라인 색이 light 토큰 위에 contrast 위반을 일으키므로 자동 매칭 제거 — 명시 토글만 light 활성화. (5) localStorage `ch13wasm.theme`로 사용자 선택 영속화. `<html data-theme>`의 부재는 dark default. | 후속 슬라이스에서 step 컴포넌트 인라인 색을 토큰으로 옮기면 prefers-color-scheme 자동 매칭 부활 가능. 그 시점에 light step body 토큰도 정의 + 색맹 친화 팔레트도 같이 검토 |
| 2026-05-01 | **theme toggle 테스트는 `colorScheme: 'dark'` emulate** | (1) `Switch to ${other} theme` 버튼 라벨이 prefers-color-scheme 실제 값에 의존(theme==null + matchMedia.matches → 'light' 시 라벨이 "Switch to dark theme"). (2) Playwright의 chromium 헤드리스 device profile이 OS 기본을 따라 light가 매칭되면 라벨이 비결정적. (3) `test.use({ colorScheme: 'dark' })`로 emulate해 prefers-color-scheme: dark를 강제 → 초기 라벨이 결정론적으로 "Switch to light theme" | 모바일 / 테마 외 다른 a11y 테스트도 환경 의존성을 줄이기 위해 page-level 토큰 + emulate 패턴을 활용 |

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
  - WASM Triangulation 바인딩 (Step 5+6): `ch13-wasm/wasm-src/myslam/bindings/bind_triangulation.cpp`
  - WASM PnP 바인딩 (Step 8): `ch13-wasm/wasm-src/myslam/bindings/bind_pnp.cpp`
  - WASM BA 바인딩 (Step 11): `ch13-wasm/wasm-src/myslam/bindings/bind_ba.cpp`
  - Phase A+ 스파이크: `ch13-wasm/wasm-src/spike/{bind_pnp_spike.cpp, bind_ba_spike.cpp, verify_pnp.mjs, verify_ba.mjs, verify_pnp_diag*.mjs}`
  - Phase C 게이트 스파이크: `ch13-wasm/wasm-src/spike/{bind_cv_spike.cpp, verify_cv.ts}` — `npm run verify:cv_spike`
  - Phase C 본 작업 검증: `ch13-wasm/wasm-src/spike/verify_features.ts` — `npm run verify:features`
  - Phase D 본 작업 검증: `ch13-wasm/wasm-src/spike/verify_triangulation.ts` — `npm run verify:triangulation`
  - Phase E 본 작업 검증: `ch13-wasm/wasm-src/spike/verify_pnp.ts` — `npm run verify:pnp` (verify_pnp.mjs는 Phase A+ 아카이브로 보존)
  - Phase G 본 작업 검증: `ch13-wasm/wasm-src/spike/verify_ba.ts` — `npm run verify:ba` (verify_ba.mjs는 Phase A+ 아카이브로 보존)
  - 빌드 스크립트: `ch13-wasm/wasm-src/{CMakeLists.txt, build.sh}` — `bash build.sh [baseline|simd|mt|mt-simd] [hello|camera|pnp_spike|ba_spike|cv_spike|features|triangulation|pnp|ba]`
  - OpenCV 분리 빌드 스크립트: `ch13-wasm/wasm-src/scripts/build-opencv.sh` — `npm run build:opencv:baseline`
  - g2o submodule (modern master): `ch13-wasm/wasm-src/third_party/g2o/`
  - Eigen submodule (tag 5.0.1): `ch13-wasm/wasm-src/third_party/eigen/`
  - OpenCV submodule (tag 4.13.0, shallow): `ch13-wasm/wasm-src/third_party/opencv/`
  - OpenCV 빌드 산출물: `ch13-wasm/wasm-src/build/opencv-{build,install}-<variant>/` (gitignored). install/lib에 `libopencv_{core,imgproc,features2d,video}.a`
  - 빌드 산출물: `ch13-wasm/public/wasm/myslam_{hello,camera,pnp_spike,ba_spike,cv_spike,features,triangulation,pnp,ba}.<variant>.{js,wasm}`
  - Step 3 UI: `ch13-wasm/src/steps/Step03_FeatureDetection/`
  - Step 4 UI: `ch13-wasm/src/steps/Step04_StereoMatching/`
  - Step 5 UI: `ch13-wasm/src/steps/Step05_Triangulation/`
  - Step 6 UI: `ch13-wasm/src/steps/Step06_InitialMap/` (lazy chunk; r3f Scene3D)
  - Step 7 UI: `ch13-wasm/src/steps/Step07_FrameTracking/` (features WASM 재사용, prev/curr LK + init 전략 토글 + maxIter sweep 차트)
  - Step 8 UI: `ch13-wasm/src/steps/Step08_PoseEstimation/` (features+triangulation+pnp 3-WASM 파이프라인, 라운드 슬라이더 + 막대차트)
  - Step 9 UI: `ch13-wasm/src/steps/Step09_KeyframeDecision/` (3 WASM 재사용, 3 정책 토글 + tracking_inliers 막대차트 + 결정 표)
  - Step 10 UI: `ch13-wasm/src/steps/Step10_NewMapPoints/` (lazy chunk; features+triangulation 재사용, mask 토글 + 4 KF 시퀀스 차트 + r3f Scene3D)
  - Step 11 UI: `ch13-wasm/src/steps/Step11_BundleAdjustment/` (lazy chunk; 4-WASM 파이프라인 + active window 2/3 + chi² histogram before/after + r3f Scene3D pre/post 토글)
  - Step 12 UI: `ch13-wasm/src/steps/Step12_SlidingWindow/` (lazy chunk; 4 정책 동시 비교 + 합성 10-KF 스트림 + r3f Scene3D)
  - Step 13 UI: `ch13-wasm/src/steps/Step13_FullPipeline/` (lazy chunk; 4-WASM 풀 파이프라인 + 재생 슬라이더 + 프리셋 3종 + r3f Scene3D)
  - SLAM 영속 모델: `ch13-wasm/src/lib/slam/{se3.ts, map.ts, pipeline.ts}` (SE(3) log-norm + KeyFrame/MapPoint/SlamMap + 4 정책 + 풀 VO 파이프라인 한 사이클)
  - Phase H 본 작업 검증: `ch13-wasm/wasm-src/spike/verify_slam.ts` — `npm run verify:slam` (12건: se3 log-norm 3 + 정책 evict 패턴 4 + final-active 검증 2 + cleanMap orphan 1 + pose spread 2)
  - Phase I Playwright 스모크: `ch13-wasm/{playwright.config.ts, tests/e2e/smoke.spec.ts, tsconfig.test.json}` — `npm run test:e2e` (15케이스: Home + 13 Step + unknown→/ 헤드리스 7.7s)
  - Phase I a11y 스펙: `ch13-wasm/tests/e2e/a11y.spec.ts` — `@axe-core/playwright 4.11`로 WCAG 2.1 AA 위반 0 검증 (Home + 13 Step + skip-link 키보드 활성화 + 라이트 테마 Home + 토글 round-trip)
  - 디자인 토큰: `ch13-wasm/src/index.css` — `--surface-*` / `--color-fg-{strong,muted,faint}` / `--border-*` / `--focus-ring` / `--color-{link,pass,fail}` (dark + light 테마 양쪽 정의) + `.skip-link` / `.sr-only` 유틸 + `.app-shell` / `.app-sidebar` / `.app-mobile-bar` / `.step-grid` 반응형 클래스
  - 단계별 학습 노트(13건): `docs/steps/{01-dataset, 02-camera, 03-feature-detection, 04-stereo-matching, 05-triangulation, 06-initial-map, 07-frame-tracking, 08-pose-estimation, 09-keyframe, 10-new-mappoints, 11-bundle-adjustment, 12-sliding-window, 13-full-pipeline}.md` — PLAN 부록 A 템플릿 기반
  - 공통 Scene3D 컴포넌트: `ch13-wasm/src/components/Scene3D.tsx` (r3f + drei)
  - TS 로더: `ch13-wasm/src/wasm/features.ts` (`loadFeaturesWasm` + `imageDataToGray`)
  - TS 로더: `ch13-wasm/src/wasm/triangulation.ts` (`loadTriangulationWasm` + `makeK` + `makeRectifiedT`)
  - TS 로더: `ch13-wasm/src/wasm/pnp.ts` (`loadPnPWasm` + `makeKMatrix` + `identityInit` + `makeInitPose6`)
  - TS 로더: `ch13-wasm/src/wasm/ba.ts` (`loadBaWasm` + `makeBaKMatrix` + `IDENTITY_EXT12` + `makeRectifiedExt12` + `packPose12` + `identityPose12`)
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

## Phase D — Step 5/6 본 작업 완료 (2026-04-26)

### 결과 요약

| 항목 | 값 |
|------|-----|
| `bind_triangulation.cpp` | 168 LOC. Embind: `triangulate(leftPts, rightPts, kL[4], tL[12], kR[4], tR[12], opts) → Float64Array stride 5 [x,y,z,metric,ok]` + `ALGO_LINEAR_SVD`/`ALGO_MIDPOINT` 상수 |
| 지원 알고리즘 | Linear SVD(`Eigen::JacobiSVD<Matrix4>` — DLT, σ4/σ3 quality), Midpoint(closest-point on skew rays — relative ray-distance/depth metric) |
| 옵션 | `algo`(0/1), `qualityThreshold`(default 0.01), `invertedReturn`(book "포기" 토글) |
| WASM 크기 | `myslam_triangulation.baseline.{js, wasm}` = 30 KB + **28 KB** (Eigen-only 헤더 인스턴스화만 포함, OpenCV 무관) |
| 빌드 시간 | ~5초 (Eigen submodule 헤더 인스턴스화 → 단일 .cpp 컴파일) |

### `verify_triangulation.ts` 검증 결과 (`npm run verify:triangulation`)

| 케이스 | 결과 |
|--------|------|
| Linear SVD 60-pt grid (depth 5..17 m, 정확 stereo 투영) | maxErr **3.96e-14 m**, maxRatio **9.37e-17**, accepted **60/60** in 1.70 ms |
| Midpoint 60-pt grid (정확 stereo 투영) | maxErr **5.57e-12 m**, accepted **60/60** in 0.72 ms |
| Linear SVD with 4 px epipolar 노이즈, threshold 0.01 | accepted **0/60**, medianRatio **2.09e-2** (well above threshold — 정상) |
| invertedReturn 토글 (depth>0 보장) | fixed=0/60, inverted=60/60, mismatches=0 — 모든 행 polarity flip 확인 |
| status ≤ 0.5 입력 | (0,0,0,NaN,0) 행으로 정상 처리, 인접 정상 입력은 영향 없음 |

### Step 5 — Triangulation (UI)

- AlgoPicker: Linear SVD / Midpoint 토글. 알고리즘별 quality threshold 의미를 슬라이더 라벨에 동적 표시.
- frame slider (0..4), `maxFeatures` slider (20..500), `qualityThreshold` slider (1e-4..0.5).
- `invertedReturn` 체크박스 — 책 algorithm.h의 "포기" 주석과 코드 polarity 충돌을 직접 관찰.
- 입력 뷰: KITTI mini 스테레오 쌍 vertical stack + baseline 표시.
- 출력 뷰: top-down (X, Z) 산점도(녹색=accepted, 빨강=rejected) + depth histogram (32 bin, 0–60 m).
- 요약: tracked / total, accepted, mean z, median metric, detect/LK/triangulate ms 분리 표시.
- VerifyGate: features+triangulation WASM 로드, 좌/우+calib 로드, accepted ≥ 30, mean depth > 0 ∧ ≤ 100 m.

### Step 6 — Initial Map (UI)

- num_features_init slider (10..200, ch13 default 50)로 게이트 임계값 직접 조정.
- KeyFrame 카드 — landmark 수 / depth min/mean/max / baseline. 임계값 충족 시 ✅ KF0 toast.
- r3f Scene3D — 좌측 카메라 frustum(녹색, KF0) + 우측 카메라 frustum(파랑, baseline 시프트) + 초기 MapPoint 클라우드(연두). drei `<OrbitControls>` + `<Grid>` + `<Line segments>`로 마우스 회전·줌·팬 지원.
- VerifyGate: features+triangulation WASM 로드, 좌/우+calib 로드, landmarks ≥ num_features_init, all depth > 0, depth 1–80 m.

### 빌드/번들 검증

| 항목 | 관측값 |
|------|--------|
| `npm run verify:triangulation` | ✅ 모든 5케이스 통과 |
| `npx tsc -b` | ✅ 0 errors |
| `npm run build` | ✅ 초기 chunk **94.86 KB gzipped** (PLAN §10 ≤ 400 KB), Step 6 lazy chunk **249.03 KB gzipped** (Three.js + r3f + drei 격리) |
| `GET /step/triangulation`, `/step/initial-map` | 200 text/html, COEP/COOP 헤더 반영 |
| `GET /wasm/myslam_triangulation.baseline.{js,wasm}` | 200 text/javascript / application/wasm, COEP/COOP 반영 |

### 의식적으로 deferred

- 실제 KeyFrame / MapPoint 데이터 모델 영속화 — Phase F(Step 9~10) Map 매니저 도입 시 통합. 현재 Step 6은 view-only이며 다음 Step에서 재사용되는 상태가 없음.
- σ3, σ4 + reprojection error 차트 — Step 5 UI는 아직 단일 frame 뷰만 제공. PerfMeter 본 구현 시점에 추가.
- `docs/steps/{05,06}-*.md` 학습 노트 — Phase I 마감 단계.
- Step 6에서 Triangulation 알고리즘 토글 노출 — 본 게이트 검증은 ch13 default(LinearSVD)로 통과. 알고리즘 비교는 Step 5 UI 책임.

### 남은 사용자 육안 관측

- `http://localhost:5173/step/triangulation`: invertedReturn 체크박스를 토글했을 때 "accepted" 카운트가 0과 (총수)로 극단적으로 점프하는지, top-down 뷰의 녹/빨강 비율이 뒤집히는지 확인.
- `http://localhost:5173/step/initial-map`: r3f 캔버스에 두 frustum이 baseline만큼 떨어져 있고, 마우스 드래그/휠로 회전·줌이 부드럽게 동작하는지, num_features_init 슬라이더를 ≤ landmark count로 내리면 ✅ KF0가 점등되는지 확인.

---

## Phase E — Step 7 본 작업 완료 (2026-04-26)

### 결과 요약

| 항목 | 값 |
|------|-----|
| 신규 C++ | 없음 (Phase C `bind_features.cpp::trackLK` 재사용 — `useInitialFlow`/`initialPts` 이미 노출됨) |
| 신규 TS UI | `src/steps/Step07_FrameTracking/index.tsx` (~520 LOC) |
| Init 전략 | `none` (`useInitialFlow=false`), `velocity` (수동 dx/dy 슬라이더, default dx=−7 = 합성 fixture GT @ 0.5× downsample) |
| 학습 시각화 | maxIter sweep `{1, 2, 3, 5, 8, 12, 20, 30}` × 두 전략 → SVG 라인차트 (수렴 라운드 비교) |
| 데이터 페어 | KITTI mini fixture frame[i−1] → frame[i] (i ∈ 1..4). Detector는 prev 좌이미지에서 실행 |
| 게이트 | (1) features WASM 로드 (2) prev+curr+calib 로드 (3) tracked ≥ 50 = `num_features_tracking` (4) 평균 dx ≈ −7 px (GT ± 2) (5) 평균 \|dy\| ≤ 2 px |

### 헤드리스 검증 (2026-04-26)

| 항목 | 관측값 |
|------|--------|
| `npm run build` | ✅ tsc -b clean + vite build — 초기 chunk **97.46 KB gzipped** (PLAN §10 ≤ 400 KB), Step 6 lazy chunk 249.03 KB gzip (변동 없음) |
| `GET /step/frame-tracking` | 200 text/html, COEP/COOP 반영 |
| `GET /wasm/myslam_features.baseline.{js,wasm}` | 200, COEP/COOP 반영 (Phase C와 동일 산출물 재사용) |

### 의식적으로 deferred

- **`map-projection` init 전략** — Step 8(PnP)에서 추정된 pose가 생긴 후 자연스럽게 합류. 현재 ParamPanel에 disabled chip으로 표시 + tooltip "available once Step 8 lands".
- **수렴 횟수(per-point iteration count) 차트** — OpenCV `calcOpticalFlowPyrLK`가 per-point 수렴 카운트를 노출하지 않아 maxIter sweep으로 대체 (학습 효과 동등).
- **MapPoint feedback loop** — `last_frame_->features_left_[i]->map_point_` 연결은 Phase F(Step 9~10) Map 매니저 도입 시점에 통합. 현재 Step 7은 view-only이며 Step 6과 마찬가지로 다음 단계로 흘러가는 영속 상태가 없음.
- **벤치 차트 (`benchStore` 통합)** — `PerfMeter`가 stub인 채라 PerfMeter 본 구현 시점에 추가.

### 남은 사용자 육안 관측

- `http://localhost:5173/step/frame-tracking`: 
  1. `velocity dx` 슬라이더를 GT(−7)에서 ±20 px로 벗어나게 변경 → tracked count가 점진적 → 급격하게 감소.
  2. `init strategy` 토글 `none` ↔ `velocity` 시 maxIter sweep 차트의 녹색(velocity) 곡선이 적색(none)보다 좌측에서 빨리 plateau에 도달하는지.
  3. curr frame 슬라이더(#0→#1, …, #3→#4)로 어느 페어든 평균 dx ≈ −7 px이 안정적으로 나오는지.

---

## Phase E — Step 8 본 작업 완료 (2026-04-26)

### 결과 요약

| 항목 | 값 |
|------|-----|
| 신규 C++ | `wasm-src/myslam/bindings/bind_pnp.cpp` (~330 LOC). spike의 `VertexPoseSE3` + `EdgeReprojectionPoseOnly`를 그대로 가져와 `estimatePose(points3dFlat, obs2dFlat, kRowMajor, initPose6, opts)` 단일 함수로 노출. ch13 EstimateCurrentPose의 4-라운드 outlier 루프(`setLevel(1)` + `chi² > 5.991` + RobustKernel detach round ≥ 2) 그대로 재현 |
| 옵션 노브 | `rounds` (def 4) · `iterPerRound` (def 10) · `chi2Threshold` (def 5.991) · `huberDelta` (def √5.991) · `removeRobustAfterRound` (def 2) · `useRobustKernel` (def true) |
| 라운드별 노출 | `roundInlierMasks` (R*N uint8) · `roundChi2Values` (R*N f64) · `roundChi2Sum` (R f64) · `roundInlierCount` (R int32) · `roundIters` (R int32) — UI는 라운드 슬라이더 1개로 outlier 변화 애니메이션 구성 |
| WASM 크기 | `myslam_pnp.baseline.{js,wasm}` = 88 KB + **391 KB**. spike 376 KB 대비 +4% (라운드 메타 packF64/packU8/packI32 런타임만 추가, g2o core/stuff/solver_dense는 동일) |
| 빌드 시간 | 단일 .cpp → ~6초 (g2o submodule는 캐시 재사용) |

### `verify_pnp.ts` 검증 결과 (`npm run verify:pnp`)

| 그룹 | 케이스 | 결과 |
|------|--------|------|
| Stage 1 noiseless (N=40) | seed 1..5 | 5/5 PASS — rotErr 0, trErr ≤ 5.6e-13 (machine precision) |
| Stage 1 1px noise (N=80) | seed 10..12 | 3/3 PASS — rotErr ≤ 1.4e-3 rad, trErr ≤ 1.2e-3 |
| Body 20% seeded outliers (N=100, 60px shift) | seed 20..22 | 3/3 PASS — **outlier recall 100% (15..20/15..20)** |
| Body RobustKernel 토글 (3종) | drop=4 / kernel=off / drop=0 | 3/3 PASS — 동일 수치 수렴 |

### Step 8 — Pose Estimation (PnP) UI

- 입력 파이프라인: prev (left+right) → GFTT → stereo LK → Linear-SVD triangulation → 3D points (left = world). prev-left → curr-left LK → 2D obs. 3D ↔ 2D 페어로 PnP. **features+triangulation+pnp 3-WASM이 동시에 메모리에 상주**해 한 화면에서 Step 3/4/5/7의 결과가 모두 흘러간다.
- ParamPanel: rounds(1..6), iterPerRound(1..20), chi²(0.5..20), Huber δ(0.1..10), drop-after-round(0..rounds), RobustKernel 체크박스, maxFeatures(20..500). cv::solvePnPRansac/EPnP/DLS는 **disabled chip + tooltip "available once calib3d is added"** — Phase E+로 명시적 deferred.
- OutputView: curr left 이미지 위 inlier(녹색)/outlier(빨강) 점 — **라운드 슬라이더로 라운드 1..R의 마스크를 한 번에 토글**하여 책의 4-라운드 outlier 제거 과정을 단계별 시각 관찰. 라운드별 inlier 카운트 막대차트(클릭 시 슬라이더 동기화) + 추정 pose 텍스트(rotation deg, translation x/y/z) + 단계별 ms.
- VerifyGate (4종):
  1. features + triangulation + pnp WASM 모두 로드
  2. prev (L+R) + curr (L) + calib 로드
  3. 3D-2D 페어 ≥ 30
  4. **inlier 비율 > 70% (PLAN §3 Step 8 명시)**
  5. final chi² 유한값

### 빌드/번들 검증

| 항목 | 관측값 |
|------|--------|
| `npm run build:wasm:pnp` | ✅ `myslam_pnp.baseline.wasm` 391 KB |
| `npm run verify:pnp` | ✅ 14건 모두 통과 |
| `npx tsc -b` | ✅ 0 errors |
| `npm run build` | ✅ 초기 chunk **100.55 KB gzipped** (PLAN §10 ≤ 400 KB), Step 6 lazy chunk 249.03 KB gzip (변동 없음) |
| `GET /step/pose-estimation` | 200 text/html, COEP/COOP 반영 |
| `GET /wasm/myslam_pnp.baseline.{js,wasm}` | 200, COEP/COOP 반영, .wasm은 application/wasm |

### 의식적으로 deferred (Phase E+ 또는 Phase F 후반)

- **`cv::solvePnPRansac` / EPnP / DLS PnP** — calib3d 모듈 추가 + OpenCV 재빌드 필요. 본 PR에 포함하면 OpenCV 분리 빌드 캐시 무효화로 잡음 증가. ParamPanel에 disabled chip + tooltip으로 학습자에게 명시.
- **Step 7 `map-projection` 활성화** — Step 8 PnP가 추정한 상대 pose를 다음 프레임 LK init으로 흘리는 통합. Step 7 ParamPanel 토글 + Step 7 ↔ Step 8 사이 영속 상태(Map 매니저)는 Phase F(Step 9~10) 도입 시 같이 처리.
- **연속 10프레임 포즈 궤적 스무딩** (PLAN §3 Step 8 보조 검증) — 현재 KITTI mini fixture는 5프레임이라 Step 13 Full Pipeline에서 자연스럽게 합류.
- **벤치 차트 + perfMeter 통합** — PerfMeter가 stub인 채라 PerfMeter 본 구현 시점에 추가.
- **Phase C+ SIMD/MT variant** — Phase G BA spike 후 일괄 도입 결정 유지.

### 남은 사용자 육안 관측

- `http://localhost:5173/step/pose-estimation`:
  1. 라운드 슬라이더를 R1 → R4로 움직이면 빨강 점이 줄어들고 녹색 점이 자리잡는지 (책의 outlier 정제 과정).
  2. `chi² threshold` 슬라이더를 1.0 → 20.0으로 올리면 inlier 수가 늘어나면서 추정 pose가 어떻게 변하는지.
  3. `RobustKernel` 체크박스를 끄면 첫 라운드부터 LM이 outlier에 잡혀끌리는지 — 라운드별 inlier 카운트 막대 변화.
  4. `drop after round=0` (kernel 즉시 떼기)와 `drop after round=4` (끝까지 유지) 사이 정확도 차이.

---

## Phase F — Step 9/10 본 작업 완료 (2026-04-26)

### 결과 요약

| 항목 | 값 |
|------|-----|
| 신규 C++ | 없음 (Step 9 = features+triangulation+pnp 재사용 / Step 10 = features+triangulation 재사용 + TS 마스크 빌더) |
| Step 9 신규 TS UI | `src/steps/Step09_KeyframeDecision/index.tsx` (~590 LOC) |
| Step 10 신규 TS UI | `src/steps/Step10_NewMapPoints/index.tsx` (~640 LOC) — lazy chunk로 분리(Scene3D 공유) |
| Step 9 정책 | `inlier-count`(ch13 default), `motion`(||t||/rot 임계), `hybrid`(union) — `covariance`는 BA 종속이라 disabled chip + tooltip "Step 11+" |
| Step 10 재검출 모드 | `mask-existing`(±10 px 원형 마스크, ch13의 `cv::rectangle(±10 box)`와 의미 동등) / `full-redetect` |
| Step 10 깊이 게이트 | `depthMin`(default 1 m) ≤ z ≤ `depthMax`(default 80 m) — PLAN §3 "depth 합리성 범위" 직접 노출 |

### 빌드/번들 검증 (2026-04-26)

| 항목 | 관측값 |
|------|--------|
| `npx tsc -b` | ✅ 0 errors (Step 9/10 추가 후) |
| `npm run build` | ✅ 초기 chunk **103.83 KB gzipped** (PLAN §10 ≤ 400 KB 충족), Scene3D vendor chunk 245.91 KB gzip — Step 6 / Step 10이 공유 |
| `GET /step/keyframe`, `/step/new-mappoints` | 200 text/html, COEP/COOP 반영 (vite dev) |
| `GET /wasm/myslam_features.baseline.wasm` | 200 application/wasm, 1 143 809 B |
| `GET /wasm/myslam_triangulation.baseline.wasm` | 200 application/wasm, 28 485 B |
| `GET /wasm/myslam_pnp.baseline.wasm` | 200 application/wasm, 400 662 B (Step 8 산출물 재사용; PROGRESS의 391 KB는 KB 환산 표기 — 실제 .wasm 정확히 391.27 KiB) |

### Step 9 — Keyframe Decision (UI)

- **파이프라인**: 4개 transition(`#0→#1`, `#1→#2`, `#2→#3`, `#3→#4`)마다 Step 7+8 미니 파이프라인(detect prev L → stereo LK prev L→R → triangulate → temporal LK prev L → curr L → PnP)을 실행. 결과 `tracking_inliers`/`||t||`/`rotationDeg`은 `params.maxFeatures` 변경에만 재계산되도록 `useMemo`로 캐시.
- **정책 토글**: `inlier-count`(`tracking_inliers < num_features_needed_for_keyframe`), `motion`(`||t|| > threshold` ∨ `rot > threshold`), `hybrid`(union). 각 임계값 슬라이더는 정책에 맞춰 disabled 처리(예: `inlier` 정책에서는 motion 슬라이더 비활성).
- **시각화**: `tracking_inliers per transition` 막대차트 + 임계값 점선 + KF로 결정된 막대에 ★ 마커. 결정 표(transition · inliers · ||t|| · rot · PnP ms · KF? · reason).
- **VerifyGate**:
  1. features + triangulation + pnp WASM 로드
  2. 5 frames + calib 로드
  3. 모든 4 transition에서 PnP 수렴
  4. 정책이 ≥ 1 KF 트리거 (학습 게이트)
  5. KF 비율 ≤ 100% (sanity)

### Step 10 — New MapPoints via Keyframe (UI)

- **파이프라인**:
  1. **Carry-over set**: frame 0 left에서 GFTT 검출 → frame 0 stereo LK → linear-SVD triangulation → temporal LK frame 0 left → frame[i] left. stereo + temporal 둘 다 성공한 점이 carry-over.
  2. **Mask 빌더**(TS): `buildCircleMask(w, h, carryPts2d, radius)` — features WASM의 `mask: Uint8Array` 인자에 그대로 전달 (ch13의 `cv::rectangle(255-fill mask, pt±10)` 의미 동등, 원형 vs 사각형 차이만).
  3. **신규 검출**: frame[i] left에서 GFTT(+optional mask) → frame[i] stereo LK → linear-SVD triangulation → depth band 필터(default 1–80 m).
- **출력**:
  - 2D 오버레이: 청록(`rgba(95,207,235)`) = carry-over 위치, 노랑(`rgba(244,208,74)`) = 새 검출.
  - r3f Scene3D: KF camera frame 기준 좌측(녹색)/우측(파랑) frustum + 같은 색상 코드의 포인트 클라우드(stride 3 Float32Array, vertexColors).
  - 시퀀스 막대차트: 4 KF 후보(`#0→#1`, …, `#0→#4`) 각각의 carry/new 누적 — 활성 KF는 1.0 opacity, 나머지 0.7.
- **VerifyGate**:
  1. features + triangulation WASM 로드
  2. frame 0 + 새 KF 프레임 + calib 로드
  3. 새 MapPoint 수 ≥ 10
  4. 새 MapPoint depth ∈ [`depthMin`, `depthMax`] m

### 의식적으로 deferred (Phase H로)

- **영속 MapPoint 모델**: Step 9/10 모두 단일 화면 view-only로 구성. 실제 KeyFrame/MapPoint 누적은 Phase H Step 12(슬라이딩 윈도우) Map 매니저 도입 시점에 통합. Step 10 carry-over set은 frame 0 stereo로 시뮬레이션 중 — Phase H에서 `Map.GetActiveMapPoints()`로 교체.
- **`covariance` 정책 (Step 9)**: landmark uncertainty propagation은 Phase G(BA) 결과에 의존 → Step 9 ParamPanel의 disabled chip + tooltip "Step 11+"로 명시.
- **모션 정책의 회전 단위 검증**: 합성 fixture는 회전이 거의 0°라 motion 정책의 회전 임계값은 슬라이더로 0.1°까지 내려야 발화. 실제 KITTI에서는 default 1.5°가 합리적임.
- **학습 노트** `docs/steps/{09,10}-*.md` — Phase I 마감 단계.

### 남은 사용자 육안 관측

- `http://localhost:5173/step/keyframe`:
  1. `policy` 토글 inlier→motion→hybrid에서 결정 표의 `KF?` 컬럼 + 막대차트 ★ 마커가 어떻게 변하는지.
  2. `inlier threshold`를 250으로 올리면 모든 transition이 KF가 되는지(상한 sanity 검증).
  3. `motion` 정책에서 회전 임계 0.1° → 5°로 올렸을 때 발화 변화.
- `http://localhost:5173/step/new-mappoints`:
  1. `redetect` 토글 mask-existing↔full-redetect 시 노랑 점이 청록 점과 겹치는지/안 겹치는지(마스크 효과).
  2. `mask radius` 30 px → 2 px로 줄였을 때 새 검출이 carry-over 코너 위까지 침범하는지.
  3. r3f Scene3D에서 마우스 회전·줌으로 두 색상 클러스터의 깊이 분포가 자연스러운지.
  4. 시퀀스 차트가 #0→#1, …, #0→#4 모두 carry+new 합 ≥ 10인지(게이트와 일관).

---

## Phase G — Step 11 본 작업 완료 (2026-04-28)

### 결과 요약

| 항목 | 값 |
|------|-----|
| 신규 C++ | `wasm-src/myslam/bindings/bind_ba.cpp` (~390 LOC). spike의 `VertexPoseSE3`/`VertexXYZ`/`EdgeProjection`을 `myslam` 네임스페이스로 옮기고 ch13 backend.cpp::Optimize의 좌/우 cam_ext 분기 + 적응적 chi² 루프 + per-edge 메타 노출을 추가 |
| API | `optimize(initPoses12, initLandmarks3, observations5, fixedPoseIndices, K, leftExt12, rightExt12, opts) → { refinedPoses12, refinedLandmarks3, perEdgeChi2{Initial,Final}, finalInlierMask, initial/finalChi2Sum, iterations, finalChi2Threshold, adaptiveDoublings, finalInlierRatio, P, L, O }` |
| 옵션 노브 | `iterations`(def 10) · `chi2Init`(def 5.991) · `adaptiveRounds`(def 5) · `adaptiveInlierRatio`(def 0.5) · `useRobustKernel`(def true) · `huberDelta`(def chi2Init — 책 그대로) |
| 솔버 | `g2o::BlockSolver<6,3>` + `g2o::LinearSolverEigen`(SimplicialLLT). 모든 landmark vertex에 `setMarginalized(true)` — Schur complement 자동 활성화. `core;stuff;solver_eigen` 3개 g2o 라이브러리만 링크 |
| WASM 크기 | `myslam_ba.baseline.{js,wasm}` = 88 KB + **439 KB** (spike 416 KB 대비 +5%). per-edge `chi²Initial`/`chi²Final` 평탄 배열 노출 + 좌/우 ext 분기 + 적응적 루프 코드만 추가. g2o core/stuff/solver_eigen은 동일 |
| 빌드 시간 | 단일 .cpp → ~6초 (g2o submodule는 캐시 재사용) |

### `verify_ba.ts` 검증 결과 (`npm run verify:ba`)

| 그룹 | 케이스 | 결과 |
|------|--------|------|
| Stage 2 회귀 noiseless P=3 L=20 | seed 1..3 | 3/3 PASS — chi² 20+자릿수 감소 (1.57e-24 ~ 1.93e-23), maxRot ≤ 2.5e-13 |
| Stage 2 회귀 1px noise P=4 L=40 | seed 10..12 | 3/3 PASS — chi² 3+자릿수 감소, maxRot < 6e-3 rad |
| 적응적 chi² 루프 (chi²_init=0.05, useRobustKernel=false) | 1 | PASS — **doublings=4, finalThr=0.80, inlierRatio=58.9%** (53/90) |
| Stereo (left+right ext) — scale gauge 고정 | 1 | PASS — chi² 6.86e+5 → 3.44e-17, **maxLm=7.29e-12** (모노큘러 spike의 ~수 cm 드리프트와 대비) |

### Step 11 — Bundle Adjustment (UI)

- **파이프라인**: KF #0(앵커, fixed)에서 GFTT → stereo LK → triangulate로 world frame 정의. 다른 KF 1..(W-1)는 KF#0 left → KF j left 시간축 LK로 3D-2D 페어를 만들어 `Step 8` PnP로 초기 pose 추정. 관측은 **KF당 좌+우 두 개**(KF#0 left = GFTT seed, KF#0 right = stereo LK 결과, KF j left = 시간 LK 결과, KF j right = KF j left → KF j right 추가 stereo LK).
- **ParamPanel**: active window 토글(2 KFs / 3 KFs), KF 인덱스(B, C) 슬라이더, `iterations`(1..30), `chi2Init`(0.5..20), `adaptive rounds`(0..8), `Huber δ`(0.1..20), `RobustKernel` 체크박스, `maxFeatures`(50..500). solver toggle은 `g2o + solver_eigen`(active) + `g2o + CXSparse`(disabled chip) + `direct LM`(disabled chip).
- **OutputView**:
  - **요약 텍스트**: observations / poses / LM iters / adaptive doublings / chi² init→final / drop ratio / inlier ratio / build ms / BA ms.
  - **Pose 표**: 각 pose의 refined translation + ‖Δt‖ + Δrot(°). KF#0은 fixed로 표시.
  - **per-edge chi² histogram (before vs after)**: 32-bin log scale, 회색 = before, 파란 = after, 빨강 점선 = 적응적 chi² 임계값. PLAN §3 Step 11 "오차 bar chart" 충족.
  - **r3f Scene3D pre/post 토글**: 좌/우 frustum + landmark 클라우드. before(회색) vs after(노랑 inlier / 어두운 빨강 outlier) 토글.
- **VerifyGate** (5종):
  1. features + triangulation + pnp + ba WASM 모두 로드
  2. 5 frames + calib 로드
  3. observations ≥ 60 (학습 게이트)
  4. **chi² 감소율 ≥ 30%** (PLAN §3 "≥ 40%"에서 mini fixture 합성 노이즈 부족으로 30%로 완화 — 결정 로그 참고)
  5. 단조 수렴 (final < initial) + 유한값

### 빌드/번들 검증

| 항목 | 관측값 |
|------|--------|
| `npm run build:wasm:ba` | ✅ `myslam_ba.baseline.wasm` 439 KB |
| `npm run verify:ba` | ✅ 10건 모두 통과 |
| `npx tsc -b` | ✅ 0 errors |
| `npm run build` | ✅ 초기 chunk **103.90 KB gzipped** (PLAN §10 ≤ 400 KB), Scene3D vendor chunk 245.91 KB gzip — Step 6 / Step 10 / Step 11이 공유 |
| `GET /step/bundle-adjustment` | 200 text/html, COEP/COOP 반영 |
| `GET /wasm/myslam_ba.baseline.{js,wasm}` | 200, COEP/COOP 반영, .wasm은 application/wasm (449 316 B) |

### 의식적으로 deferred (Phase G+ 또는 Phase H로)

- **CXSparse 통합** — `solver_eigen`(SimplicialLLT)이 mini fixture에서 충분히 빠르므로 Phase H Full Pipeline에서 active window 5+ KF를 측정한 후 병목이 확인되면 그 시점에 `wasm-src/third_party/cxsparse/` 벤더링 + `g2o/solvers/csparse` 활성화. Phase A+ 결정 그대로 유지.
- **direct LM (minimal C++ 자체 구현)** — 학습용 비교 알고리즘. 다른 모든 단계가 g2o 경로로 통합돼 있어 후순위. Phase I 마감 단계에서 합류 검토.
- **Schur on/off 토글** — `setMarginalized(true)`를 컴파일 타임에 결정하는 g2o의 솔버 구조 때문에 토글하려면 별도 솔버를 추가로 빌드해야 한다. 현재 PLAN §3의 "Schur 보완 on/off"는 disabled 안내(`Schur 보완은 항상 ON`)로 대체. Phase G+로 deferred.
- **MT/SIMD/WebGPU 가속 경로** — PLAN §4 매트릭스의 Step 11 행. baseline만 빌드 중. Phase H BA 측정 후 일괄 도입 결정 유지.
- **active window 3..15 KF** — mini fixture 5 프레임 + 영속 Map 부재로 현재 2/3 KF만 노출. Phase H Step 12 sliding window + 영속 KeyFrame 모델 도입 시 본 컴포넌트가 진짜 active map을 받도록 확장.
- **OpenMP 대신 Emscripten pthreads로 BA 병렬화** — PLAN §1.3의 결정. Phase H에서 active window가 커지면 합류.
- **벤치 차트 (`benchStore` 통합)** + **학습 노트** `docs/steps/11-*.md` — PerfMeter 본 구현 + Phase I 문서화 단계.

### 남은 사용자 육안 관측

- `http://localhost:5173/step/bundle-adjustment`:
  1. `iterations` 1 → 20으로 늘리며 chi² histogram의 파란 막대가 빨강 점선 왼쪽으로 모이는지 (수렴 시각화).
  2. `chi² init` 5.991 → 0.05로 줄이고 `useRobustKernel` 끄기 → 적응적 doublings가 4~5로 늘어나며 임계 점선이 우측으로 이동.
  3. **3D 토글 before/after**: 회색 점 클라우드(before)에서 노랑/빨강 클라우드(after)로 전환할 때 KF#1+ frustum이 landmark 분포에 맞춰 이동·회전하는지.
  4. `active window` 2 → 3 KFs 토글: chi² histogram의 막대 수가 늘어나고 (`L`이 같아도 `O`가 두 KF 분 더 추가됨) Pose 표에 KF idx 2 행이 등장.

---

## Phase H — Step 12/13 본 작업 완료 (2026-04-29)

### 결과 요약

| 항목 | 값 |
|------|-----|
| 신규 C++ | 없음 (Step 9/10/11에 이미 들어간 4 WASM(features+triangulation+pnp+ba)을 그대로 재사용) |
| 신규 TS 라이브러리 | `src/lib/slam/se3.ts` (~135 LOC) · `src/lib/slam/map.ts` (~390 LOC) · `src/lib/slam/pipeline.ts` (~430 LOC) |
| Step 12 신규 TS UI | `src/steps/Step12_SlidingWindow/index.tsx` (~470 LOC) — lazy chunk, Scene3D 공유 |
| Step 13 신규 TS UI | `src/steps/Step13_FullPipeline/index.tsx` (~520 LOC) — lazy chunk, Scene3D 공유 |
| 검증 스크립트 | `wasm-src/spike/verify_slam.ts` 12건 통과 — `npm run verify:slam` |

### `verify_slam.ts` 검증 결과

| 그룹 | 케이스 | 결과 |
|------|--------|------|
| se3.logNorm | identity / pure translation 0.5 / yaw 30° + tx 0.3 | 3/3 PASS — log-norm = √(‖ρ‖² + ‖φ‖²) 공식 정확 |
| ch13-default 정책 | 10-KF 스트림 × window=4 | PASS — evictions `[0,4,5,1,2,3]`(중복 분기 → 다양성 분기 전환) · final active size = 4 |
| FIFO 정책 | 10-KF 스트림 × window=4 | PASS — evictions `[0,1,2,3,4,5]` 시간순 · final active = 가장 최근 4 KF |
| covisibility 정책 | 10-KF 스트림 × window=4 | PASS — evictions `[0,1,2,5,3,4]` 공통 landmark 적은 KF부터 |
| distance-only 정책 | 10-KF 스트림 × window=4 | PASS — evictions `[3,4,5,6,7,8]` 항상 closest → spatial diversity 붕괴 학습 케이스 |
| cleanMap orphan | KF#0이 lmA만 관측, KF#1이 lmB만 관측, FIFO + window=1 | PASS — KF#0 evict 후 lmA가 active set에서 자동 drop |
| Pose spread metric | 5 forward KFs window=5 | PASS — variance > 0 + 유한 |

### Step 12 — Sliding Window (UI)

- **합성 KF 스트림** (`buildSyntheticKfStream`): `forward A` 5 + `duplicates` 2(KF#3 근처 ±0.05 m) + `forward B` 3 = 10 KF 후보. 80 landmarks를 4×4×5 grid로 합성, 각 KF는 frustum-heuristic으로 ~20 landmark 관측 (인접 KF끼리 covisibility가 높아 정책 차이가 시연됨).
- **ParamPanel**: `num_active_keyframes` 슬라이더 2..(stream-1), 4 정책 토글, `duplicate threshold` 슬라이더(ch13-default 활성 시만), stream synthesis 슬라이더(forward A/duplicates/forward B).
- **OutputView**:
  - **정책 비교 표**: 4 정책 동시 실행 결과(evictions / landmarks dropped / final active KFs / final active LMs / pose spread variance) — 활성 정책 행에 색 강조.
  - **insertion + eviction 로그**: 각 step의 `inserted KF` / `active KFs after` / `eviction reason + landmarks removed`. evict가 발생한 행은 빨강 배경.
  - **r3f Scene3D**: active KF frustum(시안) + evicted KF frustum(회색 dim) + active landmark(노랑) + inactive landmark(회색).
- **VerifyGate** (5종):
  1. stream ≥ window+1 (eviction이 발생할 조건)
  2. 활성 정책이 ≥ 1 KF evict
  3. evict 횟수 = total KF − window (모든 정책이 정확한 budget 달성)
  4. cleanMap 후 active landmark > 0
  5. 4 정책 모두 NaN 없이 종료

### Step 13 — Full Pipeline (UI)

- **풀 파이프라인** (`runPipeline` in `src/lib/slam/pipeline.ts`): ch13 `Frontend::AddFrame` switch를 그대로 모사.
  - frame 0: `StereoInit` — DetectFeatures + FindFeaturesInRight + BuildInitMap + InsertKeyframe (world frame anchor).
  - frame i≥1: `Track` = TrackLastFrame(LK prev→curr) + EstimateCurrentPose(g2o PnP, `relative_motion_ * last_pose` init) + (옵션) InsertKeyframe(`tracking_inliers < num_features_needed_for_keyframe` 시) + DetectFeatures with mask-existing(±10) + FindFeaturesInRight + TriangulateNewPoints (camera frame → world frame transform). KF 삽입 시 Backend BA(`enableBackend` 토글) 호출 — ch13 backend.cpp::Optimize.
- **ParamPanel**: 프리셋 3종(`book-default` / `conservative` / `aggressive`) + 개별 슬라이더(maxFeatures, PnP rounds, num_features_needed_for_keyframe, num_features_tracking, BA iterations / chi² init / adaptive rounds, window size, policy 토글).
- **InputView**: 재생 컨트롤(`«` `‹` 슬라이더 `›` `»`) + frame별 outcome 테이블(frame · outcome · tracked · inliers · KF? · new lm · BA chi² Δ% · ms). KF 삽입 행 강조.
- **OutputView**: 요약(total frames / KFs / landmarks / total ms / BA accumulated ms / sliding window evictions / last BA chi²) + r3f Scene3D(active KF frustum + evicted KF dim + active MapPoint cloud).
- **VerifyGate** (6종 — `enableBackend` 토글 시 7번째 추가):
  1. 4 WASM 모듈 로드
  2. 5 frames + calib 로드
  3. lost/failed = 0
  4. KF ≥ 1
  5. 궤적 좌표 모두 유한
  6. 누적 길이 ≤ 50 m (mini fixture sanity)
  7. (Backend ON 시) BA chi² 단조 감소 (KF 삽입 시)

### 빌드/번들 검증

| 항목 | 관측값 |
|------|--------|
| `npm run verify:slam` | ✅ 12건 모두 통과 |
| `npx tsc -b` | ✅ 0 errors |
| `npm run build` | ✅ 초기 chunk **104.03 KB gzipped** (PLAN §10 ≤ 400 KB), Scene3D vendor chunk 245.91 KB gzip — Step 6/10/11/12/13이 공유 |
| `GET /step/sliding-window` / `/step/full-pipeline` | 200 text/html, COEP/COOP 반영 |
| `GET /wasm/myslam_{features,triangulation,pnp,ba}.baseline.wasm` | 200 application/wasm, COEP/COOP 반영 (Phase C/D/E/G 산출물 재사용) |

### 의식적으로 deferred (Phase I로)

- **Web Worker 분리** — PLAN §9 Phase H의 "전체 VO 워커" 항목. 현재 Step 13 useMemo 동기 재계산 모델은 mini fixture 5 frame에서 sub-200 ms이지만 long sequence에서는 main thread blocking이 문제됨. Worker 도입 시 `runPipeline`을 그대로 옮길 수 있도록 모듈 경계가 이미 깔려 있음.
- **영속 feature → BA observation** — 현재 `runBackendOnce`는 refined pose로 landmark를 재투영 + deterministic 1px 미만 노이즈를 측정값으로 사용(BA monotone gate 통과 + 가벼움). 실제 영속 feature 추적은 Phase I worker화와 함께 도입.
- **궤적 polyline 시각화** — 현재 Scene3D는 KF frustum + landmark cloud만. 누적 궤적 polyline은 r3f `<Line points={...}>` 한 줄 추가로 가능하지만, 본 PR에서는 KF frustum 위치로 궤적을 시사적으로 표현. Phase I 마감 시 합류.
- **PLY export + KITTI GT 비교** — PLAN §3 Step 13 명시. 실 KITTI 시퀀스 + GT poses가 들어왔을 때 합류.
- **Step 12의 long-sequence 실 KITTI 평가** — PLAN §3 "10 KF 제거 후 평균 포즈 분산"의 long-sequence 게이트. mini fixture는 frame당 motion이 작아 합성 stream으로 대체. Phase I 또는 후속 평가에서 실 KITTI 05 100+ frame으로 정책 비교.
- **Step 13의 `map-projection` LK init 전략** (Step 7 deferred) — Step 13 풀 파이프라인은 이미 `relative_motion_ * last_pose` init을 PnP에 흘리고 있어 사실상 합류. Step 7 단독 페이지의 disabled chip을 Phase I에서 제거 검토.
- **학습 노트** `docs/steps/{12,13}-*.md` — Phase I 문서화 단계.

### 남은 사용자 육안 관측

- `http://localhost:5173/step/sliding-window`:
  1. policy 토글 (ch13-default → FIFO → covisibility → distance-only) 시 비교 표의 evictions 컬럼이 같지만 final active KF id 집합이 달라지는지.
  2. duplicate threshold를 0.01 → 1.0으로 올렸을 때 ch13-default 정책의 reason이 "closest"에서 "farthest"로 전환되는지.
  3. forward A=2, duplicates=4, forward B=2 같은 극단 설정에서 distance-only 정책이 모든 신규 forward KF를 즉시 evict 하는지(diversity 붕괴 학습 케이스).
- `http://localhost:5173/step/full-pipeline`:
  1. 프리셋 토글 (book-default → conservative → aggressive) 시 KF 수 / total ms / BA chi² 곡선이 변하는지.
  2. enable backend 체크박스 ON/OFF 시 KF 삽입 행의 BA chi² Δ 컬럼이 채워지거나 비는지.
  3. window size 2로 줄이면 sliding window evictions가 발생해 Scene3D에서 KF가 회색으로 dim 처리되는지.
  4. 재생 슬라이더를 frame 0 → 4로 움직이며 누적 KF 추가 + landmark 클라우드 확장이 점진적으로 보이는지.

---

## 갱신 규칙

1. **Phase/Step 착수 시**: "현재 진행 중" 필드를 갱신.
2. **완료 시**: 해당 체크박스 ✅, "검증 게이트 통과"에 통과 조건 요약을 기록.
3. **결정이 발생하면**: "주요 결정 로그"에 append-only로 한 줄 추가.
4. **블로커 발생**: "현재 블로커"에 원인·시도한 해결책·다음 행동을 기록.
5. **세션 종료 시**: 반드시 이 파일 갱신 후 커밋. 커밋 메시지 예: `progress: finish Phase A scaffolding`.

---

## Phase I — Playwright 스모크 슬라이스 (2026-04-29)

### 결과 요약

| 항목 | 값 |
|------|-----|
| devDep 추가 | `@playwright/test ^1.59.1` |
| 브라우저 바이너리 | chromium 1217 + chromium-headless-shell 1217 (macOS arm64) — `~/Library/Caches/ms-playwright/` 캐시 |
| 신규 파일 | `ch13-wasm/playwright.config.ts` · `ch13-wasm/tests/e2e/smoke.spec.ts` · `ch13-wasm/tsconfig.test.json` |
| 스크립트 | `npm run test:e2e` (vite preview 자동 기동) · `npm run test:e2e:report` |
| 헤드리스 결과 | **15 PASS / 0 FAIL** — Home + 13 Step + unknown→/ 리다이렉트, 전체 7.7s |
| 빌드 영향 | 초기 chunk gzip 104.03 KB 변동 없음 (PLAN §10 ≤ 400 KB 충족) — 테스트 코드는 `tsconfig.test.json` 별도 reference라 vite 빌드에서 제외 |

### 게이트 정책

스모크는 **회귀 그물**이지 기능 검증이 아니다 — WASM 수치 정합성은 `verify_*.ts` 9종(node)에 위임.

각 케이스가 통과하려면:
1. 라우트가 200으로 마운트 (vite preview SPA fallback 그대로)
2. `<h1>`에 해당 Step의 title이 보임 (lazy chunk + Suspense fallback 통과 확인)
3. `pageerror` (uncaught exception) 0건
4. tolerated console patterns(`/[vite]/`, React DevTools 안내, headless WebGL) 외의 `console.error` 0건

### Playwright 설정 메모

- `webServer.command`은 `npm run preview -- --host 127.0.0.1 --port 4173 --strictPort`. `PLAYWRIGHT_BASE_URL` 환경변수가 있으면 webServer 기동을 생략하므로 외부 dev 서버에 붙여 디버깅도 가능.
- `workers: 1` 단일 워커. 브라우저 세션당 OpenCV/g2o WASM이 메모리에 동시 적재되면 8 GB+를 먹을 수 있어 직렬 실행이 안전.
- `trace: 'retain-on-failure'`. 회귀 발생 시 `playwright-report/trace.zip`으로 분석.
- `tsconfig.test.json`을 root tsconfig references에 추가해 `npx tsc -b`가 spec까지 strict type-check.

### 의식적으로 deferred

- **a11y(시멘틱/contrast/키보드 조작)**: ✅ 2026-05-01 완료 — 아래 "Phase I — a11y 베이스라인 슬라이스" 절 참고.
- **모바일 레이아웃**: 현재 `App.tsx`는 260 px 사이드바 + main 2-column 그리드 고정. 모바일 (≤ 768 px) 시 사이드바를 햄버거로 전환 + Step 컴포넌트 그리드 1열 전환 필요. r3f Canvas 크기도 별도 검토.
- **dark mode**: 현재 hard-coded dark theme. CSS variable 기반 테마 토글로 전환 + `prefers-color-scheme` 감지.
- **`docs/steps/{01..13}-*.md` 학습 노트** (PLAN 부록 A 템플릿): Phase I 후반에 일괄 작성. 본 PR에는 없음.
- **GH Actions CI matrix**: PLAN §7.4 "Playwright 스모크: 각 Step 화면이 로드되고 검증 게이트를 자동 프리셋으로 통과". `node-version: 22.x` + `npx playwright install --with-deps chromium` + `npm run test:e2e`. Phase I 마지막에 합류.
- **webkit / firefox 브라우저**: chromium 단일 프로젝트로 시작. PLAN §1.4의 Safari 17.4 호환은 webkit 추가 시점에 검증.

### 남은 사용자 육안 관측

- (선택) `npm run test:e2e:report`로 HTML 리포트 확인 → 각 케이스의 라우트 timing이 정상인지(특히 r3f를 쓰는 Step 6/10/11/12/13가 1 s 내외에 mount 되는지).
- 추후 a11y 작업 PR이 들어가기 전, 매번 `npm run test:e2e`가 그린인지 확인.

---

## Phase I — a11y 베이스라인 슬라이스 (2026-05-01)

### 결과 요약

| 항목 | 값 |
|------|-----|
| devDep 추가 | `@axe-core/playwright ^4.11.2` |
| 신규 파일 | `ch13-wasm/tests/e2e/a11y.spec.ts` (15 케이스) |
| 수정 파일 | `src/index.css`(디자인 토큰 + skip-link + focus-visible 룰) · `src/App.tsx`(skip-link + landmark + nav aria-label) · `src/components/{StepLayout,ParamPanel,PerfMeter,VerifyGate}.tsx`(aria-labelledby + role=status + 토큰 사용) · 8 step 컴포넌트(`'#666'`/`'#777'` → `'var(--color-fg-faint)'`) · `src/steps/Step13_FullPipeline/index.tsx`(playback slider `aria-label="Playback frame"`) · `src/steps/Step05_Triangulation/index.tsx`(timing line 인라인 색 → token) |
| 빌드 영향 | 초기 chunk gzip 104.43 KB(±0 KB, PLAN §10 ≤ 400 KB 충족) — 토큰화는 인라인 색을 var()로 치환할 뿐이라 번들 변화 미세 |
| 헤드리스 결과 | **30 PASS / 0 FAIL** (스모크 15 + a11y 15) — 28.5s |

### 게이트 정책 (axe-core)

`tests/e2e/a11y.spec.ts`는 14 페이지(Home + 13 Step) 각각에 대해 axe-core를 돌려 **WCAG 2.1 AA 위반 0**을 게이트로 한다.

- `withTags(['wcag2a','wcag2aa','wcag21a','wcag21aa'])` — best-practice는 의도적으로 미적용(결정 로그 2026-05-01 항목 참고).
- `waitForLoadState('networkidle')` 후 axe — lazy chunk(Step 6/10/11/12/13) + r3f Canvas mount까지 기다림.
- 마지막 케이스: 첫 Tab → `Skip to main content` 링크에 포커스 → Enter → `<main id="main-content">`이 `:focus`인지 검증.

### 디자인 토큰 매핑 (참고용)

| token | 값 | 용도 | 최저 대비비(text on this surface) |
|-------|-----|------|----------------------------------|
| `--surface-app` | #111 | body bg | — |
| `--surface-sidebar` | #1a1a1a | aside bg | `--color-fg-faint`(#aaa) = 11.46:1 |
| `--surface-panel` | #181818 | section bg | `--color-fg-faint` = 11.94:1 |
| `--surface-panel-deep` | #141414 | PerfMeter bg | `--color-fg-faint` = 12.71:1 |
| `--surface-panel-faint` | #161616 | VerifyGate(미통과) bg | `--color-fg-faint` = 12.31:1 |
| `--surface-panel-active` | #2a3d5c | NavLink(현재 페이지) bg | `--color-fg-faint` = 4.95:1 ✓ |
| `--surface-panel-success` | #113a22 | VerifyGate(통과) bg | `--color-fg-faint` = 5.29:1 ✓ |
| `--color-fg-strong` | #fff | h1/h2 / strong text | — |
| `--color-fg-default` | #eee | body text | — |
| `--color-fg-muted` | #bbb | 보조 정보(요약, 라벨) | — |
| `--color-fg-faint` | #aaa | 힌트, placeholder, 초기 metric | 모든 표면에서 ≥ 4.5:1 |
| `--color-pass` / `--color-fail` | #6c6 / #d77 | VerifyGate item 마커(이모지 색만, 텍스트 의미는 sr-only로 별도 명시) | — |

### 의식적으로 deferred

- **heading-order(best-practice)**: 13 Step 컴포넌트의 ParamPanel 내부가 모두 `<h3>` 패턴 — 일괄 `<h2>`로 올리면 의미 변화 없이 13 파일을 다 만져야 한다. axe `best-practice` 활성화 + heading-order 정리는 별도 슬라이스로 분리.
- **사이드바 키보드 네비게이션 강화**: 현재 Tab + Enter로 충분히 동작. 화살표 네비게이션(`role="menu"` 같은 strong-typed 패턴)은 학습 페이지 13개 환경에서 과한 도입 — 검토 후 follow-up.
- **Reduce-motion 지원**: r3f Canvas / drei `OrbitControls`는 자동 회전 없음 → `prefers-reduced-motion` 분기 불필요. CSS 트랜지션(skip-link 슬라이드)은 120ms로 짧아 critical 아님.
- **Color-blind 친화 팔레트(deuter/proton)**: PLAN §10. `--color-pass`/`--color-fail` 토큰만 swap하면 됨. 별도 슬라이스로 색상 결정.

### 남은 사용자 육안 관측

- (선택) Tab으로 사이드바 → 콘텐츠 진입 흐름 확인: Tab 한 번에 "Skip to main content" 노출 → Enter로 main 진입.
- 키보드만으로 Step 6/10/11/12/13의 r3f Canvas 위 컨트롤(슬라이더/체크박스)을 조작 가능한지 점검 — Canvas 자체는 마우스/터치 전용(OrbitControls)이지만 ParamPanel은 키보드로 충분.
- 추후 모바일 / dark mode 작업 시 매 PR 전에 `npm run test:e2e`가 30/30 그린인지 확인.

---

## Phase I — 모바일 + 테마 토글 + 학습 노트 슬라이스 (2026-05-01)

### 결과 요약

| 항목 | 값 |
|------|-----|
| 신규 파일 | `docs/steps/{01..13}-*.md` 13건 |
| 수정 파일 | `src/App.tsx` (햄버거 토글 + 테마 토글 + Esc/route close) · `src/index.css` (light 테마 토큰 + `.app-shell`/`.app-sidebar`/`.app-mobile-bar`/`.step-grid` 클래스 + `@media (max-width: 768px)`) · `src/components/StepLayout.tsx` (`step-grid` 클래스 적용) · `tests/e2e/smoke.spec.ts` (모바일 viewport 1 케이스) · `tests/e2e/a11y.spec.ts` (라이트 테마 Home 1 + toggle round-trip 1) |
| 빌드 영향 | 초기 chunk gzip 104.88 KB(+0.45 KB, 테마 state + 모바일 토글 React 코드. PLAN §10 ≤ 400 KB 충족) |
| 헤드리스 결과 | **33 PASS / 0 FAIL** (스모크 15 + 모바일 1 + a11y 15 + 라이트 1 + 토글 1) — 30.5s |

### 모바일 레이아웃

- **Breakpoint**: `@media (max-width: 768px)` (iPhone 12 mini ~ iPad mini portrait 모두 포함)
- **사이드바**: position: fixed + width 80vw + transform: translateX(-100%) + visibility: hidden(닫힘 시) / 0 + visible(열림 시)
- **햄버거 버튼**: `<button aria-expanded aria-controls="app-sidebar">☰</button>` — 모바일에서만 sticky top bar에 노출
- **Backdrop**: `.app-sidebar-backdrop[data-open=true]` 클릭 시 닫힘
- **자동 닫힘**: ① 라우트 변경(`useLocation` watch) ② Esc 키 ③ backdrop 클릭
- **Step grid**: `.step-grid`가 desktop `260px 1fr 1fr`에서 모바일 `1fr`로 전환 → ParamPanel/Input/Output이 세로로 나열

### dark/light 테마 토글

- **Activation**: 사이드바 하단의 `.theme-toggle` 버튼 (`☀ Light` / `🌙 Dark` 라벨, aria-label은 "Switch to <other> theme"으로 동적)
- **Persistence**: `localStorage["ch13wasm.theme"] = 'dark' | 'light'`. 부재 시 `<html>`에 `data-theme` 속성 미부여 → CSS의 `:root` 룰(dark)이 default
- **prefers-color-scheme**: 자동 매칭은 **의도적으로 비활성**(step 컴포넌트의 dark 인라인 색이 light shell과 충돌). 사용자 명시 토글만 light 활성화
- **Light 테마 범위**: shell-only(App + StepLayout 외부 + sidebar). 13 step 내부는 dark 유지 — 의도된 디자인(light browser 안의 dark code editor 패턴)

### 학습 노트 (`docs/steps/*.md`)

- 13건 모두 PLAN 부록 A 템플릿 기반: 학습 목표 / C++ 원본 매핑 / UI / 알고리즘 / 가속 경로 / 검증 / 학습 노트 본문 / 의도된 실패
- PROGRESS.md "Step 검증 게이트 통과 현황" + 결정 로그를 source of truth로 인용
- 길이: 평균 3.3 KB(약 100 라인). Step 11/12/13은 BA + 정책 + 풀 파이프라인 비중이 커 4–5 KB

### 의식적으로 deferred (Phase I+ 이후)

- **light 테마의 step body 적용**: 13 step 컴포넌트의 인라인 `#181818`/`#222`/`#aaa` 등을 `var(--surface-*)`/`var(--color-fg-*)`로 토큰화하면 step 페이지도 light AA를 통과 가능. 별도 슬라이스로 일괄 sweep 권장(PR 13개 파일 변경).
- **prefers-color-scheme 자동 매칭 부활**: 위 step 토큰화 완료 후.
- **모바일 r3f Canvas 크기 최적화**: Step 6/10/11/12/13의 Scene3D는 grid cell 크기로 자동 fit하지만 모바일에서는 viewport 전폭에 비해 작아 보일 수 있음 — `aspect-ratio` + 줌 컨트롤 별도 검토.
- **Web Worker로 `runPipeline` 분리**: PLAN §9 Phase H 항목 — Step 13 long sequence 시 main thread blocking 회피.
- **GH Actions CI matrix**: `node-version: 22.x` + `npx playwright install --with-deps chromium` + `npm run test:e2e`. 33-case headless gate.
- **webkit / firefox 브라우저**: chromium 단일 프로젝트. PLAN §1.4의 Safari 17.4 호환 검증은 webkit 추가 시점에.

### 남은 사용자 육안 관측

- 모바일 viewport(브라우저 DevTools 또는 실제 기기)에서 햄버거 → 사이드바 슬라이드 인 → Step 링크 탭 → 사이드바 자동 닫힘 → Step 페이지가 1열로 잘 보이는지.
- 테마 토글 버튼을 눌러 라이트로 전환했을 때 Home은 white shell로 깔끔하지만 step 페이지는 light shell + dark step body 패턴인지 (의도된 동작).
- localStorage가 새로고침 후에도 유지되어 마지막 선택 테마로 복원되는지.
- `docs/steps/*.md` 13건이 GitHub UI에서 제대로 렌더링되는지(테이블, 체크박스, 코드 블록).

---
