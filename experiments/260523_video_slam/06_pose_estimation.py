# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Pose estimation per detector + trajectory plots
#
# `04_match_pairwise.py` 가 저장한 `pairwise_matches.npz` 를 detector별로 읽어
# 5-point essential matrix (RANSAC) → `recoverPose` 로 상대 pose를 추정한다.
#
# 단안(monocular)이므로 `‖t‖ = 1` (per-pair) — trajectory shape/방향만 의미.
# 실패한 pair에서는 trajectory를 segment로 끊는다.
#
# 출력
# - `data/260523_house/keyframes/trajectories/<detector>.json`
# - `data/260523_house/keyframes/trajectories/<detector>_xz.png`
# - `data/260523_house/keyframes/trajectories/comparison_xz.png` — 4-detector 비교

# %%
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from keyframe import KeyframeStore
from pose import EssentialMatrixEstimator, Trajectory, bridge_segments

HERE = Path.cwd() if "__file__" not in globals() else Path(__file__).resolve().parent
DATA_DIR = HERE.parent / "data" / "260523_house"
STORE_DIR = DATA_DIR / "keyframes"
MATCH_DIR = STORE_DIR / "matches"
OUT_DIR = STORE_DIR / "trajectories"

DETECTORS = ["sift", "orb", "akaze", "brisk"]
COLORS = {"sift": "tab:blue", "orb": "tab:orange", "akaze": "tab:green", "brisk": "tab:red"}


# %% [markdown]
# ## 셋업

# %%
OUT_DIR.mkdir(exist_ok=True)
store = KeyframeStore.load(STORE_DIR, load_features=True)
K = store.camera.K
n_frames = len(store.frames)
n_pairs = n_frames - 1
print(f"frames={n_frames}  pairs={n_pairs}")
print(f"K =\n{K}")

estimator = EssentialMatrixEstimator(threshold_px=1.0, prob=0.999,
                                     min_matches=8, min_inliers_pose=15)


# %% [markdown]
# ## Detector별 pose estimation 루프

# %%
trajectories: dict[str, Trajectory] = {}
trajectories_bridged: dict[str, Trajectory] = {}

for det in DETECTORS:
    t0 = time.perf_counter()

    # cache keypoint xy arrays for this detector
    kp_xy = [kf[det].xy for kf in store.frames]

    # consolidated matches
    with np.load(MATCH_DIR / det / "pairwise_matches.npz") as z:
        offsets = z["offsets"]
        pairs_idx = z["pairs"]

    pair_results = []
    for i in range(n_pairs):
        lo, hi = int(offsets[i]), int(offsets[i + 1])
        qt = pairs_idx[lo:hi]
        if len(qt) == 0:
            # fabricate a "failed" RelativePose-equivalent by feeding empty arrays
            pts_a = np.zeros((0, 2), np.float32)
            pts_b = np.zeros((0, 2), np.float32)
        else:
            pts_a = kp_xy[i][qt[:, 0]]
            pts_b = kp_xy[i + 1][qt[:, 1]]
        rp = estimator.estimate(pts_a, pts_b, K, frame_a=i, frame_b=i + 1)
        pair_results.append(rp)

    traj = Trajectory.from_pairs(
        detector=det, method=estimator.method, camera=store.camera,
        frame_ids=[kf.id for kf in store.frames], pairs=pair_results,
    )
    trajectories[det] = traj

    dt = time.perf_counter() - t0
    s = traj.summary()
    print(
        f"  {det:5s}  success={s['n_success_pairs']:>3d}/{s['n_total_pairs']}  "
        f"valid_frames={s['n_valid']:>3d}/{s['n_frames']}  "
        f"segments={s['n_segments']:>2d}  longest={s['longest_segment']:>3d}  "
        f"({dt:.1f}s)"
    )

    json_path = OUT_DIR / f"{det}.json"
    traj.save(json_path)
    print(f"    -> {json_path.relative_to(STORE_DIR)}")

    # Bridge across failed pairs — splice segments into a common world frame
    # where possible. Saves alongside the raw trajectory so the two are
    # directly comparable.
    bridged, attempts = bridge_segments(traj, store)
    n_gaps = len(traj.segments) - 1
    n_ok = bridged.extra["n_bridges"]
    n_groups = len(set(bridged.extra["world_group"]))
    print(
        f"    bridge: {n_ok}/{n_gaps} gaps closed -> "
        f"{n_groups} world group(s) (from {len(traj.segments)} segments)"
    )
    bridged_path = OUT_DIR / f"{det}_bridged.json"
    bridged.save(bridged_path)
    print(f"    -> {bridged_path.relative_to(STORE_DIR)}")
    trajectories_bridged[det] = bridged


# %% [markdown]
# ## 개별 trajectory plot (XZ — top-down)
#
# OpenCV 카메라 좌표계: X 우, Y 아래, Z 앞.
# 카메라가 거의 수평으로 움직였다면 XZ 투영이 자연스러운 top-down view.

# %%
def plot_xz_segments(ax, traj: Trajectory, color: str, label: str | None = None) -> None:
    drew_label = False
    for seg in traj.segment_positions():
        if len(seg) < 2:
            continue
        l = label if (label and not drew_label) else None
        ax.plot(seg[:, 0], seg[:, 2], color=color, linewidth=1.4, alpha=0.9, label=l)
        ax.plot(seg[0, 0], seg[0, 2], "o", color=color, markersize=4)
        ax.plot(seg[-1, 0], seg[-1, 2], "s", color=color, markersize=4)
        drew_label = True


for det, traj in trajectories.items():
    fig, ax = plt.subplots(figsize=(7, 7))
    plot_xz_segments(ax, traj, COLORS[det])
    ax.set_aspect("equal")
    ax.set_xlabel("X (unit-norm cumulative)")
    ax.set_ylabel("Z (unit-norm cumulative)")
    s = traj.summary()
    ax.set_title(
        f"{det.upper()} trajectory — XZ top-down\n"
        f"{s['n_segments']} segments, longest={s['longest_segment']} frames, "
        f"success {s['n_success_pairs']}/{s['n_total_pairs']}"
    )
    ax.grid(alpha=0.3)
    out = OUT_DIR / f"{det}_xz.png"
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"  {det:5s} -> {out.name}")


# %% [markdown]
# ## Bridged trajectory plot (XZ — top-down)
#
# 같은 `world_group` 에 속한 segment들은 같은 색으로 묶고, bridged gap 은 dotted
# connector 로 잇는다. Bridge 가 성공할수록 색 묶음이 적어지고, dotted 선이
# 부드럽게 이어지는 모양이 된다.

# %%
def plot_xz_bridged(ax, traj: Trajectory) -> None:
    """Color segments by world_group, connect bridged segments with dotted lines."""
    groups = traj.extra.get("world_group", list(range(len(traj.segments))))
    cmap = plt.get_cmap("tab20")
    color_of_group: dict[int, tuple] = {}
    for k, (s, e) in enumerate(traj.segments):
        if e - s < 2:
            continue
        g = groups[k]
        if g not in color_of_group:
            color_of_group[g] = cmap(len(color_of_group) % cmap.N)
        c = color_of_group[g]
        seg = traj.positions[s:e]
        ax.plot(seg[:, 0], seg[:, 2], color=c, linewidth=1.4, alpha=0.95)
        ax.plot(seg[0, 0], seg[0, 2], "o", color=c, markersize=4)
        ax.plot(seg[-1, 0], seg[-1, 2], "s", color=c, markersize=4)
        # dotted connector across the bridged invalid frame to the next segment
        # in the same world group (if any).
        if k + 1 < len(traj.segments) and groups[k + 1] == g:
            nxt_s = traj.segments[k + 1][0]
            a = traj.positions[e - 1]
            b = traj.positions[nxt_s]
            if np.isfinite(a).all() and np.isfinite(b).all():
                ax.plot([a[0], b[0]], [a[2], b[2]],
                        color=c, linewidth=0.9, linestyle=":", alpha=0.7)


for det, bridged in trajectories_bridged.items():
    raw = trajectories[det]
    fig, axes = plt.subplots(1, 2, figsize=(14, 7), sharex=False, sharey=False)
    plot_xz_segments(axes[0], raw, COLORS[det])
    plot_xz_bridged(axes[1], bridged)

    s_raw = raw.summary()
    n_groups = len(set(bridged.extra["world_group"]))
    n_bridges = bridged.extra["n_bridges"]

    for ax in axes:
        ax.set_aspect("equal")
        ax.set_xlabel("X (unit-norm cumulative)")
        ax.set_ylabel("Z (unit-norm cumulative)")
        ax.grid(alpha=0.3)
    axes[0].set_title(
        f"{det.upper()} — raw\n"
        f"{s_raw['n_segments']} segments, "
        f"success {s_raw['n_success_pairs']}/{s_raw['n_total_pairs']}"
    )
    axes[1].set_title(
        f"{det.upper()} — bridged\n"
        f"{n_bridges}/{s_raw['n_segments'] - 1} gaps closed, "
        f"{n_groups} world group(s)"
    )

    out = OUT_DIR / f"{det}_bridged_xz.png"
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"  {det:5s} -> {out.name}")


# %% [markdown]
# ## 비교 plot — 4 detectors on one figure
#
# Segment 사이에는 world frame이 일치하지 않으니, 모든 segment를 각자의 origin에서
# 시작하도록 그려 모양 비교만 한다.

# %%
fig, axes = plt.subplots(1, 2, figsize=(14, 7))

for det, traj in trajectories.items():
    plot_xz_segments(axes[0], traj, COLORS[det], label=det.upper())

axes[0].set_aspect("equal")
axes[0].set_xlabel("X")
axes[0].set_ylabel("Z")
axes[0].set_title("XZ top-down — all detectors, all segments overlaid")
axes[0].grid(alpha=0.3)
axes[0].legend(loc="upper right", fontsize=9)

# Y vs frame index — elevation profile, lets us see vertical motion / drift
for det, traj in trajectories.items():
    y = traj.positions[:, 1]
    axes[1].plot(np.arange(len(y)), y, color=COLORS[det],
                 linewidth=1.2, alpha=0.85, label=det.upper())
axes[1].set_xlabel("frame index")
axes[1].set_ylabel("Y (unit-norm cumulative)")
axes[1].set_title("Vertical drift — Y component vs. frame")
axes[1].grid(alpha=0.3)
axes[1].legend(loc="upper right", fontsize=9)

fig.tight_layout()
cmp_path = OUT_DIR / "comparison_xz.png"
fig.savefig(cmp_path, dpi=130)
plt.show()
print(f"comparison -> {cmp_path}")


# %% [markdown]
# ## Round-trip sanity

# %%
reloaded = Trajectory.load(OUT_DIR / "sift.json")
print(f"reload OK: {reloaded.detector} {reloaded.method}  "
      f"valid={int(reloaded.valid.sum())}/{len(reloaded.frame_ids)}  "
      f"segments={len(reloaded.segments)}")
