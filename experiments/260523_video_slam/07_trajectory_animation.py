# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Frame-by-frame trajectory animation
#
# `Trajectory.poses_wc` 를 frame index 순서대로 누적하며,
# 현재 frame 이미지(좌)와 누적 trajectory(우)를 함께 보여주는 MP4 를 만든다.
#
# 좌측: 현재 frame + 해당 detector의 keypoint overlay
# 우측: XZ top-down trajectory — 전체 segment를 faint background로 깔고,
#       frame i 까지의 누적 polyline + 현재 카메라 위치 marker를 강조
#
# Trajectory가 segment로 끊겨 있어도 `positions` 배열에 NaN이 끼어 있어
# `Line2D` 가 자동으로 break — 별도 처리 불필요.
#
# 출력
# - `data/260523_house/keyframes/trajectories/<detector>_bridged_animated.mp4`
# - `data/260523_house/keyframes/trajectories/<detector>_bridged_animation_final.png`

# %%
import time
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

from keyframe import KeyframeStore
from pose import Trajectory

HERE = Path.cwd() if "__file__" not in globals() else Path(__file__).resolve().parent
DATA_DIR = HERE.parent / "data" / "260523_house"
STORE_DIR = DATA_DIR / "keyframes"
TRAJ_DIR = STORE_DIR / "trajectories"

DETECTOR = "akaze"        # change to "sift" / "orb" / "brisk" to render others
FPS = 15                  # output video frame rate
IMG_DOWNSCALE = 3         # 1080x1920 -> 360x640 for the left panel
KP_DOWNSCALE_RADIUS = 4   # keypoint circle radius in the downsampled image


# %% [markdown]
# ## 데이터 로드

# %%
store = KeyframeStore.load(STORE_DIR, load_features=True)
traj = Trajectory.load(TRAJ_DIR / f"{DETECTOR}_bridged.json")
positions = traj.positions  # (N, 3) NaN where invalid
xs, zs = positions[:, 0], positions[:, 2]
n = len(traj.frame_ids)
assert n == len(store.frames)
print(f"loaded {n} frames, detector={DETECTOR}, "
      f"valid={int(traj.valid.sum())}, segments={len(traj.segments)}")


# %% [markdown]
# ## 이미지 prefetch (LRU 대신 단순 디스크 캐시; n=321 충분히 작음)

# %%
def load_frame_small(idx: int) -> np.ndarray:
    """Return RGB image downsampled by IMG_DOWNSCALE."""
    path = DATA_DIR / store.frames[idx].image_path
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    h, w = img.shape[:2]
    img = cv2.resize(img, (w // IMG_DOWNSCALE, h // IMG_DOWNSCALE),
                     interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def overlay_keypoints(img_rgb: np.ndarray, kf, detector: str) -> np.ndarray:
    """Draw keypoints onto a downsampled image (in pixel coords of full-res)."""
    xy = kf[detector].xy / IMG_DOWNSCALE
    out = img_rgb.copy()
    for x, y in xy:
        cv2.circle(out, (int(round(x)), int(round(y))), KP_DOWNSCALE_RADIUS,
                   (0, 255, 0), 1, lineType=cv2.LINE_AA)
    return out


print("pre-rendering frames (image + keypoints) ...")
t0 = time.perf_counter()
frame_images = [overlay_keypoints(load_frame_small(i), store.frames[i], DETECTOR)
                for i in range(n)]
print(f"  done in {time.perf_counter() - t0:.1f}s "
      f"({frame_images[0].shape}, {len(frame_images)} frames, "
      f"{sum(im.nbytes for im in frame_images) / 1e6:.0f} MB)")


# %% [markdown]
# ## Figure setup

# %%
fig = plt.figure(figsize=(11, 6.5))
gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.4])

ax_img = fig.add_subplot(gs[0, 0])
ax_traj = fig.add_subplot(gs[0, 1])

ax_img.set_axis_off()
img_artist = ax_img.imshow(frame_images[0])
title_artist = ax_img.set_title(
    f"{store.frames[0].id}   ({DETECTOR.upper()})", fontsize=10
)

# Faint full trajectory in the background (so viewer sees where it's going).
ax_traj.plot(xs, zs, color="0.85", linewidth=1.0, zorder=1)

# Active "grown so far" polyline and current camera marker.
(line_active,) = ax_traj.plot([], [], color="tab:green", linewidth=1.6, zorder=2)
(marker_curr,) = ax_traj.plot([], [], "o", color="tab:red", markersize=8, zorder=3)

ax_traj.set_aspect("equal")
ax_traj.set_xlabel("X (unit-norm cumulative)")
ax_traj.set_ylabel("Z (unit-norm cumulative)")
ax_traj.grid(alpha=0.3)
margin = 1.5
finite = np.isfinite(xs) & np.isfinite(zs)
ax_traj.set_xlim(xs[finite].min() - margin, xs[finite].max() + margin)
ax_traj.set_ylim(zs[finite].min() - margin, zs[finite].max() + margin)

status_text = ax_traj.text(
    0.02, 0.97, "", transform=ax_traj.transAxes, va="top", fontsize=9,
    bbox=dict(facecolor="white", alpha=0.85, edgecolor="0.7"),
)

fig.tight_layout()


# %% [markdown]
# ## Frame update

# %%
def update(i: int):
    img_artist.set_data(frame_images[i])
    title_artist.set_text(f"{store.frames[i].id}   ({DETECTOR.upper()})")

    line_active.set_data(xs[: i + 1], zs[: i + 1])

    if traj.valid[i]:
        marker_curr.set_data([xs[i]], [zs[i]])
        # find which segment we're in for the status text
        seg_idx = next(
            (k for k, (s, e) in enumerate(traj.segments) if s <= i < e), -1
        )
        seg_info = f"seg {seg_idx + 1}/{len(traj.segments)}" if seg_idx >= 0 else "—"
    else:
        marker_curr.set_data([np.nan], [np.nan])
        seg_info = "INVALID"

    valid_so_far = int(traj.valid[: i + 1].sum())
    status_text.set_text(
        f"frame {i + 1:>3d}/{n}\n"
        f"valid {valid_so_far:>3d}\n"
        f"{seg_info}"
    )
    return img_artist, title_artist, line_active, marker_curr, status_text


# %% [markdown]
# ## Render to MP4

# %%
print(f"rendering animation @ {FPS} fps ...")
t0 = time.perf_counter()

anim = animation.FuncAnimation(fig, update, frames=n, interval=1000 / FPS, blit=False)
out_path = TRAJ_DIR / f"{DETECTOR}_bridged_animated.mp4"
writer = animation.FFMpegWriter(fps=FPS, codec="libx264",
                                extra_args=["-pix_fmt", "yuv420p", "-preset", "fast"])
anim.save(str(out_path), writer=writer, dpi=120)
plt.close(fig)
print(f"  done in {time.perf_counter() - t0:.1f}s -> {out_path}")


# %% [markdown]
# ## Final-frame still

# %%
update(n - 1)
final_path = TRAJ_DIR / f"{DETECTOR}_bridged_animation_final.png"
# re-create figure since plt.close above closed it; quicker: re-draw via savefig
# We already closed it, so build a small static plot at the end state.
fig_static, (ax_i, ax_t) = plt.subplots(1, 2, figsize=(11, 6.5),
                                        gridspec_kw={"width_ratios": [1.0, 1.4]})
ax_i.imshow(frame_images[-1])
ax_i.set_axis_off()
ax_i.set_title(f"{store.frames[-1].id}   ({DETECTOR.upper()})", fontsize=10)
ax_t.plot(xs, zs, color="tab:green", linewidth=1.4)
ax_t.plot(xs[traj.valid][0], zs[traj.valid][0], "o", color="black",
          markersize=6, label="start")
last_valid = np.where(traj.valid)[0][-1]
ax_t.plot(xs[last_valid], zs[last_valid], "s", color="tab:red",
          markersize=7, label="end")
ax_t.set_aspect("equal")
ax_t.set_xlabel("X")
ax_t.set_ylabel("Z")
ax_t.grid(alpha=0.3)
ax_t.legend(loc="best", fontsize=9)
ax_t.set_title(
    f"Final state — {int(traj.valid.sum())}/{n} valid, "
    f"{len(traj.segments)} segments"
)
fig_static.tight_layout()
fig_static.savefig(final_path, dpi=130)
plt.close(fig_static)
print(f"final still -> {final_path}")
