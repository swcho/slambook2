# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # SIFT — frame-to-frame 2D-2D pose estimation
#
# 인접 프레임 SIFT 매칭 → essential matrix (RANSAC) → R, t 복원.
#
# **주의 — monocular 한계**
# - 각 pair마다 t는 **unit norm** (scale ambiguity, 미터 단위 아님)
# - R 누적은 의미 있음 (회전은 metric)
# - 누적 trajectory는 scale 없이는 의미 없음 → 본 노트에서는 그리지 않음
#
# **주의 — 내참 K**
# - 캘리브레이션 파일이 없어 `fx=fy=image_width`, principal point = image center 가정
# - 실제 calibration이 있으면 아래 `K` 를 교체하세요
#
# Outputs
# - `data/20fps_real_img_archieve/sift_pose/poses.csv`
# - `data/20fps_real_img_archieve/sift_pose/pose_summary.png`

# %%
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

HERE = Path.cwd() if "__file__" not in globals() else Path(__file__).resolve().parent
IMG_DIR = HERE.parent / "data" / "20fps_real_img_archieve"
FEAT_DIR = IMG_DIR / "sift_feat"
OUT_DIR = IMG_DIR / "sift_pose"

RATIO = 0.75
RANSAC_THRESH_PX = 1.0   # essential matrix inlier threshold (pixels)
RANSAC_PROB = 0.999
MIN_MATCHES_FOR_E = 8    # 5-point algorithm minimum is 5, but >=8 for stability


# %% [markdown]
# ## 내참 행렬 (assumed)

# %%
W, H = 640, 480
FX = FY = float(W)
CX, CY = W / 2.0, H / 2.0
K = np.array([[FX, 0, CX],
              [0, FY, CY],
              [0,  0,  1]], dtype=np.float64)
print("Assumed K =\n", K)


# %% [markdown]
# ## 헬퍼

# %%
def load_feat(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """returns (kpts_xy (N,2), descriptors (N,128))."""
    with np.load(path) as z:
        kp = z["keypoints"].astype(np.float32)
        desc = z["descriptors"].astype(np.float32)
    return kp[:, :2], desc


def matched_points(kp_a: np.ndarray, desc_a: np.ndarray,
                   kp_b: np.ndarray, desc_b: np.ndarray,
                   matcher: cv2.BFMatcher,
                   ratio: float = RATIO) -> tuple[np.ndarray, np.ndarray]:
    if len(desc_a) < 2 or len(desc_b) < 2:
        return np.zeros((0, 2), np.float32), np.zeros((0, 2), np.float32)
    knn = matcher.knnMatch(desc_a, desc_b, k=2)
    good = [(m.queryIdx, m.trainIdx) for m, n in knn if m.distance < ratio * n.distance]
    if not good:
        return np.zeros((0, 2), np.float32), np.zeros((0, 2), np.float32)
    q_idx = np.array([g[0] for g in good])
    t_idx = np.array([g[1] for g in good])
    return kp_a[q_idx], kp_b[t_idx]


def rot_to_euler_deg(R: np.ndarray) -> tuple[float, float, float]:
    """ZYX (yaw, pitch, roll) in degrees — small-angle interpretable."""
    sy = float(np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    if sy > 1e-6:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0.0
    return float(np.degrees(z)), float(np.degrees(y)), float(np.degrees(x))


# %% [markdown]
# ## 셋업

# %%
OUT_DIR.mkdir(exist_ok=True)
feat_paths = sorted(FEAT_DIR.glob("scene_avg_*.npz"))
assert len(feat_paths) >= 2, f"need >=2 feature files in {FEAT_DIR}"
print(f"found {len(feat_paths)} feature files")

matcher = cv2.BFMatcher(cv2.NORM_L2)


# %% [markdown]
# ## Pose estimation 루프
#
# 각 pair에 대해 `findEssentialMat` (RANSAC) → `recoverPose` (cheirality check).

# %%
rows: list[dict] = []

kp_prev, desc_prev = load_feat(feat_paths[0])
for i in range(len(feat_paths) - 1):
    kp_curr, desc_curr = load_feat(feat_paths[i + 1])
    pts_a, pts_b = matched_points(kp_prev, desc_prev, kp_curr, desc_curr, matcher)
    n_good = len(pts_a)

    row = {
        "pair_idx": i,
        "frame_a": feat_paths[i].stem,
        "frame_b": feat_paths[i + 1].stem,
        "n_good": n_good,
        "n_inliers_E": 0,
        "n_inliers_pose": 0,
        "success": False,
        "tx": np.nan, "ty": np.nan, "tz": np.nan,
        "yaw_deg": np.nan, "pitch_deg": np.nan, "roll_deg": np.nan,
    }

    if n_good >= MIN_MATCHES_FOR_E:
        E, mask_e = cv2.findEssentialMat(
            pts_a, pts_b, K,
            method=cv2.RANSAC, prob=RANSAC_PROB, threshold=RANSAC_THRESH_PX,
        )
        if E is not None and E.shape == (3, 3):
            row["n_inliers_E"] = int(mask_e.sum())
            n_pose, R, t, mask_pose = cv2.recoverPose(E, pts_a, pts_b, K, mask=mask_e)
            row["n_inliers_pose"] = int(n_pose)
            row["success"] = n_pose >= 5
            if row["success"]:
                tt = t.ravel()
                row["tx"], row["ty"], row["tz"] = float(tt[0]), float(tt[1]), float(tt[2])
                row["yaw_deg"], row["pitch_deg"], row["roll_deg"] = rot_to_euler_deg(R)

    rows.append(row)
    if (i + 1) % 10 == 0 or i == len(feat_paths) - 2:
        ok = "OK " if row["success"] else "FAIL"
        print(f"  [{i+1:>3}/{len(feat_paths)-1}] {ok}  "
              f"good={n_good:>3}  inl_E={row['n_inliers_E']:>3}  "
              f"inl_pose={row['n_inliers_pose']:>3}")
    kp_prev, desc_prev = kp_curr, desc_curr


# %% [markdown]
# ## CSV 저장

# %%
csv_path = OUT_DIR / "poses.csv"
fieldnames = list(rows[0].keys())
with csv_path.open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fieldnames)
    w.writeheader()
    w.writerows(rows)
print(f"csv -> {csv_path}")


# %% [markdown]
# ## 요약
#
# - 성공률 (cheirality 통과)
# - per-pair inlier 분포
# - translation 방향 (unit vector 성분 시계열)
# - per-pair rotation magnitude (Euler norm)

# %%
arr = np.array([[r["pair_idx"], r["n_good"], r["n_inliers_E"], r["n_inliers_pose"],
                 1.0 if r["success"] else 0.0,
                 r["tx"], r["ty"], r["tz"],
                 r["yaw_deg"], r["pitch_deg"], r["roll_deg"]] for r in rows])

n_pairs = len(rows)
n_ok = int(arr[:, 4].sum())
print(f"success: {n_ok}/{n_pairs} ({100*n_ok/n_pairs:.1f}%)")
print(f"inliers_pose  median={int(np.median(arr[:, 3]))}  "
      f"min={int(arr[:, 3].min())}  max={int(arr[:, 3].max())}")

ok_mask = arr[:, 4] == 1.0
rot_mag = np.linalg.norm(arr[ok_mask, 8:11], axis=1)
print(f"rotation/pair median={np.median(rot_mag):.2f} deg  "
      f"max={rot_mag.max():.2f} deg")

# %%
fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

axes[0].plot(arr[:, 0], arr[:, 1], label="good matches", color="gray", linewidth=1)
axes[0].plot(arr[:, 0], arr[:, 2], label="inliers (E)", color="tab:blue", linewidth=1)
axes[0].plot(arr[:, 0], arr[:, 3], label="inliers (pose)", color="tab:green", linewidth=1)
axes[0].axhline(MIN_MATCHES_FOR_E, color="red", linestyle="--", alpha=0.4, label=f"min={MIN_MATCHES_FOR_E}")
axes[0].set_ylabel("# matches")
axes[0].set_title(f"Match / inlier counts (success {n_ok}/{n_pairs})")
axes[0].legend(loc="upper right", fontsize=8)
axes[0].grid(alpha=0.3)

# translation direction (unit) — NaN where failed
axes[1].plot(arr[:, 0], arr[:, 5], label="tx", alpha=0.8)
axes[1].plot(arr[:, 0], arr[:, 6], label="ty", alpha=0.8)
axes[1].plot(arr[:, 0], arr[:, 7], label="tz", alpha=0.8)
axes[1].set_ylabel("t (unit, per pair)")
axes[1].set_title("Translation direction — monocular, unit norm only")
axes[1].legend(loc="upper right", fontsize=8)
axes[1].grid(alpha=0.3)

# rotation magnitude per pair (Euler-norm proxy)
axes[2].plot(arr[:, 0], np.linalg.norm(arr[:, 8:11], axis=1),
             color="tab:purple", linewidth=1)
axes[2].set_xlabel("pair index (i -> i+1)")
axes[2].set_ylabel("‖Euler‖ (deg)")
axes[2].set_title("Per-pair rotation magnitude")
axes[2].grid(alpha=0.3)

fig.tight_layout()
png_path = OUT_DIR / "pose_summary.png"
fig.savefig(png_path, dpi=120)
plt.show()
print(f"plot -> {png_path}")
