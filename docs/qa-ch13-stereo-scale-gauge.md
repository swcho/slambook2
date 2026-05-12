# ch13의 실제 파이프라인에서는 scale gauge 문제가 어떻게 해결되는가?

> 선행 문서: [qa-noiseless-ba-scale-gauge.md](qa-noiseless-ba-scale-gauge.md)
> — 모노큘러 noiseless BA에서 chi²는 0인데 maxTr만 살아 있는 현상의 원인 분석.

## 짧은 답

**Stereo baseline이 절대 scale을 박아준다.**
ch13 backend의 BA는 모노큘러가 아니라 **stereo BA**다. 모든 landmark가 가능하면 left/right 두 카메라 모두로 reprojection edge를 갖고, 두 카메라 사이의 외부 파라미터 `_cam_ext` (baseline ≈ 0.54 m for KITTI)는 **상수로 박혀** 있다. 이 상수가 cost function에 들어가는 순간 sim(3) 작용 중 scale 1-DOF가 불변성을 깨고 cost에 잡히게 된다 → gauge group이 sim(3) → SE(3)으로 한 차원 줄어든다.

따라서 Phase G의 실제 구현에서는 [선행 문서](qa-noiseless-ba-scale-gauge.md)에서 다룬 “chi² ≈ 0인데 maxTr만 큰” 모노큘러 scale drift 패턴이 나타나지 않는다.

---

## 1. 어디서 scale이 박히는가 — 코드 추적

### 1.1 데이터셋: KITTI calib에서 metric extrinsic 추출

[`ch13/src/dataset.cpp:22-43`](../ch13/src/dataset.cpp)

```cpp
for (int i = 0; i < 4; ++i) {
    // ... read 3x4 projection matrix P_i = K_i [R_i | t_i] ...
    Vec3 t;
    t << projection_data[3], projection_data[7], projection_data[11];
    t = K.inverse() * t;                  // ← metric translation in meters
    K = K * 0.5;                          // image was downsampled 2×
    Camera::Ptr new_camera(new Camera(
        K(0,0), K(1,1), K(0,2), K(1,2),
        t.norm(),                         // ← baseline (m)
        SE3(SO3(), t)));                  // ← fixed extrinsic
    cameras_.push_back(new_camera);
}
```

KITTI calib의 `P_1 = K [I | -K·b·e_x]` 에서 `K⁻¹·t = -b·e_x` 가 나오고, 그 norm이 곧 stereo baseline (KITTI 기준 ≈ 0.537 m). 이 값이 카메라 객체의 `pose()` 안에 **고정 상수**로 들어간다.

### 1.2 Backend: edge 생성 시 left/right extrinsic 주입

[`ch13/src/backend.cpp:74-98`](../ch13/src/backend.cpp)

```cpp
// K 와 좌우 외부 파라미터
Mat33 K = cam_left_->K();
SE3 left_ext  = cam_left_->pose();    // ← 고정 (보통 identity)
SE3 right_ext = cam_right_->pose();   // ← 고정 (baseline 박힌 SE3)

for (auto &landmark : landmarks) {
    // ...
    for (auto &obs : observations) {
        EdgeProjection *edge = nullptr;
        if (feat->is_on_left_image_) {
            edge = new EdgeProjection(K, left_ext);   // ← 좌측 edge
        } else {
            edge = new EdgeProjection(K, right_ext);  // ← 우측 edge
        }
        // ... addEdge ...
    }
}
```

같은 landmark가 좌·우 모두에서 관측되었으면 (대부분의 경우) **edge가 두 개씩** 생성된다. 좌측 edge는 `left_ext`를, 우측 edge는 `right_ext`를 자기 측정 모델에 끼워 넣는다.

### 1.3 EdgeProjection: projection chain에 `_cam_ext` 가 박혀 있다

[`ch13/include/myslam/g2o_types.h:104-121`](../ch13/include/myslam/g2o_types.h)

```cpp
class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ> {
    EdgeProjection(const Mat33 &K, const SE3 &cam_ext) : _K(K) {
        _cam_ext = cam_ext;       // ← 변수가 아니라 상수로 보관
    }

    virtual void computeError() override {
        SE3 T  = v0->estimate();                    // body pose (변수)
        Vec3 pw = v1->estimate();                   // landmark   (변수)
        Vec3 pos_pixel = _K * (_cam_ext * (T * pw));// ← 변환 체인
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }
};
```

핵심은 `_cam_ext * (T * pw)`. Projection chain이 **`world → body → physical-camera → pixel`** 구조이고, `body → physical-camera` 단계가 **상수**다.

### 1.4 초기 landmark도 metric으로 박혀 들어옴

[`ch13/src/frontend.cpp:353-387` `BuildInitMap()`](../ch13/src/frontend.cpp)

```cpp
std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
// ...
triangulation(poses, points, pworld);  // ← stereo triangulation
// pworld 는 metric (m 단위)
```

첫 keyframe의 map point들은 **metric baseline에 의한 stereo triangulation**으로 만들어진다. 이 시점에서 “scale 1”의 의미가 절대 좌표계에 못박힌다.

---

## 2. 왜 baseline 상수가 scale gauge를 깨는가 — 수식

### 2.1 모노큘러 reprojection의 scale-invariance (복습)

선행 문서 §4에서 본 그대로:

$$
\pi(R_i, \mathbf{t}_i, \mathbf{p}_j) \;=\; K\,\frac{R_i \mathbf{p}_j + \mathbf{t}_i}{[R_i \mathbf{p}_j + \mathbf{t}_i]_z}
$$

치환 $(\mathbf{t}_i, \mathbf{p}_j) \mapsto (s\mathbf{t}_i, s\mathbf{p}_j)$ 를 대입하면 분자·분모가 동시에 $s$ 배 → **불변**. 이게 모노큘러의 1-DOF scale gauge.

### 2.2 Stereo edge의 reprojection — `t_ext`가 안 따라간다

ch13 stereo edge는

$$
\pi_{\text{right}}(\theta) \;=\; K\,\frac{R_{\text{ext}}(R_i \mathbf{p}_j + \mathbf{t}_i) + \mathbf{t}_{\text{ext}}}{\big[R_{\text{ext}}(R_i \mathbf{p}_j + \mathbf{t}_i) + \mathbf{t}_{\text{ext}}\big]_z}
$$

여기서 $(R_{\text{ext}}, \mathbf{t}_{\text{ext}}) = $ `right_ext`는 **변수가 아니다**. 모노큘러 scale 작용 $(\mathbf{t}_i, \mathbf{p}_j) \mapsto (s\mathbf{t}_i, s\mathbf{p}_j)$ 을 대입하면

$$
\pi'_{\text{right}} \;=\; K\,\frac{s\,R_{\text{ext}}(R_i \mathbf{p}_j + \mathbf{t}_i) + \mathbf{t}_{\text{ext}}}{\big[s\,R_{\text{ext}}(R_i \mathbf{p}_j + \mathbf{t}_i) + \mathbf{t}_{\text{ext}}\big]_z}
$$

분자·분모가 **동시에는** $s$ 배 안 된다 (상수항 $\mathbf{t}_{\text{ext}}$ 가 안 따라가기 때문). 따라서

$$
\pi'_{\text{right}}(\theta) \;\neq\; \pi_{\text{right}}(\theta) \quad \text{whenever}\quad \mathbf{t}_{\text{ext}} \neq 0.
$$

→ stereo edge의 residual이 $s$ 변화에 **반응한다** → chi²가 scale 오차를 “본다” → 더 이상 무료(gauge)가 아니다.

> 왜 left edge 하나만으로는 부족한가? `left_ext` 가 보통 identity (translation 0)라 `t_ext_left = 0`. 위 인자 분리에서 분자·분모가 정확히 $s$ 배가 되어 다시 invariant. **scale을 박는 건 baseline ≠ 0 인 우측 edge다.**

### 2.3 Hessian 관점

선행 문서 §2의 표를 ch13 backend 기준으로 다시 적으면:

| 구성 | gauge group | gauge DOF | null(H) |
|------|-------------|-----------|---------|
| 모노큘러 reproj only | sim(3) | 7 | 7 |
| **Stereo reproj (ch13)** | **SE(3)** | **6** | **6** |
| Stereo + pose 1개 fixed | trivial | 0 | 0 |

stereo edge가 들어오는 순간 **null 공간이 7 → 6으로 줄어든다**. 줄어든 한 차원이 정확히 그 “모든 t와 p에 곱해지는 단일 양의 상수” 방향이고, 그 방향은 `t_ext` 상수항이 cost에 새로 만들어내는 jacobian 성분에 의해 정확히 채워진다.

---

## 3. ch13 backend에서 SE(3) gauge는 어떻게 처리되는가

여기서 한 가지 짚어둘 미묘한 점:

[`ch13/src/backend.cpp:55-68`](../ch13/src/backend.cpp) 의 keyframe 정점 추가 루프에는 **`setFixed(true)` 호출이 없다.** 즉 stereo가 scale 1-DOF를 잡아도 **SE(3) 6-DOF (origin pose) 자유도는 형식상 남아 있다.**

| 잠긴 DOF | 모노큘러 | ch13 stereo |
|---------|---------|-------------|
| 0개 | gauge DOF = 7 | gauge DOF = 6 |
| pose 1개 fixed | 1 (scale) | 0 |
| **0개 (ch13 실제 코드)** | 7 ❌ | **6** — 형식상 자유, 실제로는 LM 댐핑 + 외부 anchor로 안정 |

ch13의 실 운영 시 origin gauge가 폭주하지 않는 이유:
- **Active window 외부의 keyframe pose**가 implicit anchor 역할. Backend는 active window만 최적화하지만, 그 window 안의 pose들은 이전 active window가 끝나면서 외부 좌표계에 “찍힌” 상태로 시작한다.
- LM의 댐핑 $\lambda$ 가 null-space 방향의 step을 0 근처로 잡아둔다.
- Frontend의 `EstimateCurrentPose` (PnP)가 새 keyframe pose를 매번 외부 map에 정렬해서 넘기므로, backend가 받는 초기값 자체가 절대 좌표계에서 잘 정의되어 있다.

> 즉 ch13는 **scale gauge는 stereo로 정확히 잡고**, **SE(3) gauge는 implicit 하게 (앵커 keyframe들의 정보 흐름으로) 잡는다**. 이 둘을 혼동하지 않는 것이 중요하다.

진짜로 SE(3) gauge까지 explicit 하게 잡고 싶다면 `vertex_pose->setFixed(true)` 를 가장 오래된 keyframe에 한 번 걸어주면 충분하다 (stereo 덕분에 scale은 이미 잡혔으니 pose 1개 fix 만으로 모든 gauge가 닫힘).

---

## 4. Phase G 실제 구현에서 보이는 양상

| 진단 항목 | 모노큘러 noiseless BA | **ch13 stereo BA (Phase G)** |
|---------|----------------------|------------------------------|
| chi² | 1.2e-22 | ~1e-22 |
| maxRot | 3.1e-09 | ~1e-09 |
| **maxTr** | **8.7e-04** ⚠️ | **~1e-09** ✅ |
| sim(3) 정렬 후 maxTr | ≈ chi² 수준으로 떨어짐 | (정렬 불필요) 처음부터 같은 수준 |
| $\|\mathbf{t}^{\text{est}}\|/\|\mathbf{t}^{\text{gt}}\|$ | 1.0008 정도 | 1.0 (numerical) |

stereo BA에서는 **chi², maxRot, maxTr이 같은 자릿수**까지 같이 떨어진다. “비대칭 잔차” signature가 사라진다.

---

## 5. 처방 표 ch13 관점에서 다시 보기

선행 문서 §6의 처방 표를 ch13에 매칭:

| 방법 | ch13가 채택? | 비고 |
|------|-------------|------|
| Pose 1개 fix | ❌ (코드상 안 함) | stereo 덕분에 필수는 아님; explicit하게 origin을 잠그고 싶다면 추가 가능 |
| **Stereo edge 사용** | ✅ **자동** | `EdgeProjection(K, right_ext)` 가 baseline을 박음 |
| RGB-D edge | ❌ | 데이터셋이 KITTI stereo |
| Distance prior | ❌ | 불필요 |
| sim(3) gauge prior | ❌ | 불필요 |

→ ch13는 처방 중 **“stereo/RGB-D edge”** 한 가지로 모노큘러 scale 문제를 통째로 우회한다.

---

## 요약

- ch13 backend는 **모노큘러 BA가 아니다** — left/right 두 카메라 reprojection을 동시에 cost로 쓰는 **stereo BA**.
- 우측 edge의 `_cam_ext`에는 KITTI calib에서 추출된 **metric baseline이 상수로 박혀** 있다 (`dataset.cpp:39`, `g2o_types.h:111`).
- 이 상수가 reprojection 식의 분자·분모에 동시에 $s$ 배 되지 않는 단일 항을 만들어 → **scale 작용 하의 invariance를 깬다** → gauge group이 sim(3) → SE(3) 으로 축소 → null(H) = 7 → 6.
- 따라서 선행 문서에서 본 “chi² ≈ 0, maxRot ≈ 0, **maxTr ≠ 0**” 비대칭이 ch13 실 운영에서는 나타나지 않는다.
- SE(3) 6-DOF origin gauge는 ch13에서 explicit하게 잡지는 않지만, active-window 바깥의 keyframe pose들과 LM 댐핑이 implicit anchor 역할을 한다. explicit하게 잠그려면 가장 오래된 keyframe에 `setFixed(true)` 한 줄이면 충분하다 (stereo가 이미 scale을 잡았으므로).
- 즉 **모노큘러 scale drift 처방 7가지 중 ch13가 고른 것은 “stereo edge 사용” 하나뿐이고, 그것만으로 충분하다.**
