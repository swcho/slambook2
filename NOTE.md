Step 7 Frame Tracking UI 설계

  - 데이터 소스: 기존 KITTI mini fixture. 합성 GT는 프레임당 14px 좌측 이동(raw) → 0.5× downsample 시 7px.                                                                                        
  - 두 init 전략 (PLAN의 3종 중 map-projection은 Step 8에서 추정 pose가 생긴 후 자연스럽게 합류 → 이번엔 deferred):                                                                               
    a. none (useInitialFlow=false)                                                                                                                                                                
    b. velocity (수동 dx/dy 슬라이더, default dx=−7 = GT)                                                                                                                                         
  - 수렴 라운드 비교 차트: maxIter ∈ {1, 2, 5, 10, 20, 30} sweep, 두 전략별 추적 성공 수                                                                                                          
  - Verify gate: WASM 로드, prev/curr 프레임 로드, tracked ≥ 50 (PLAN num_features_tracking), mean dx가 GT(−7) 근방.    