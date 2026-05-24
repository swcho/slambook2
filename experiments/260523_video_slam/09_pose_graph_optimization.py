# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Sim(3) pose-graph optimization
#
# `<det>_bridged.json` (chain + bridge edges baked into the initial guess)
# 와 `loop_closures/<det>.json` (long-range edges) 를 합쳐 g2o Sim(3) PGO 로
# 최적화. 출력은 `<det>_pgo.json` + 비교 plot.
#
# Sim(3) 인 이유: monocular 의 `‖t‖ = 1` 컨벤션 때문에 loop closure 와 chain
# 의 스케일이 모순됨. SE(3) PGO 는 해결 불가; 각 노드에 스칼라 스케일 자유도가
# 있어야 reconcile 됨.
#
# 출력
# - `data/260523_house/keyframes/trajectories/<det>_pgo.json`
# - `data/260523_house/keyframes/trajectories/<det>_pgo_xz.png`

# %%
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from keyframe import KeyframeStore
from pose import Trajectory
from loop_closure import load_loop_closures
from sim3_pgo import dedupe_loop_closures, optimize_sim3, trajectory_with_optimized_poses

HERE = Path.cwd() if "__file__" not in globals() else Path(__file__).resolve().parent
DATA_DIR = HERE.parent / "data" / "260523_house"
STORE_DIR = DATA_DIR / "keyframes"
TRAJ_DIR = STORE_DIR / "trajectories"
LOOP_DIR = STORE_DIR / "loop_closures"

DETECTOR = "sift"
NMS_WINDOW = 5            # frames; loop-closure dedup neighborhood
ROBUST_DELTA = 2.6        # ≈ sqrt(7) — 1-sigma for 7-DoF Sim3 residual
ITERATIONS = 30


# %% [markdown]
# ## 입력 로드

# %%
store = KeyframeStore.load(STORE_DIR, load_features=False)
traj = Trajectory.load(TRAJ_DIR / f"{DETECTOR}_bridged.json")
all_closures = load_loop_closures(LOOP_DIR / f"{DETECTOR}.json")
print(f"bridged trajectory: {len(traj.frame_ids)} frames, "
      f"{int(traj.valid.sum())} valid, {len(traj.segments)} segments")
print(f"loop closures loaded: {len(all_closures)}")

# Dedupe — 다수의 redundant edge 가 single physical revisit 을 묘사하므로
# NMS 로 cluster 당 최강 1개만 남김.
closures = dedupe_loop_closures(all_closures, nms_window=NMS_WINDOW)
print(f"after dedup (nms={NMS_WINDOW}): {len(closures)} edges")
for c in closures:
    print(f"  {c.frame_a:>3d} <-> {c.frame_b:>3d}  sim={c.similarity:.3f}  "
          f"in={c.rp.n_inliers_pose:>4d}")


# %% [markdown]
# ## 최적화

# %%
t0 = time.perf_counter()
result = optimize_sim3(traj, closures,
                       iterations=ITERATIONS,
                       robust_delta=ROBUST_DELTA,
                       verbose=True)
dt = time.perf_counter() - t0
print()
print(f"PGO done in {dt:.2f}s")
print(f"  vertices:    {result.n_vertices}")
print(f"  edges:       chain={result.n_chain_edges}  "
      f"bridge={result.n_bridge_edges}  loop={result.n_loop_edges}")
print(f"  chi2:        {result.initial_chi2:.3e} -> {result.final_chi2:.3e}  "
      f"({100*(1-result.final_chi2/max(result.initial_chi2, 1e-12)):.1f}% reduction)")
print(f"  scale range: [{np.nanmin(result.scales[traj.valid]):.3f}, "
      f"{np.nanmax(result.scales[traj.valid]):.3f}]")


# %% [markdown]
# ## 결과 저장

# %%
traj_pgo = trajectory_with_optimized_poses(traj, result)
pgo_path = TRAJ_DIR / f"{DETECTOR}_pgo.json"
traj_pgo.save(pgo_path)
print(f"saved -> {pgo_path.relative_to(STORE_DIR)}")


# %% [markdown]
# ## 비교 plot — bridged vs PGO
#
# Loop closure 가 잡힌 frame 들을 마커로 표시.

# %%
xs_b, zs_b = traj.positions[:, 0], traj.positions[:, 2]
xs_o, zs_o = traj_pgo.positions[:, 0], traj_pgo.positions[:, 2]

fig, axes = plt.subplots(1, 2, figsize=(14, 7))

axes[0].plot(xs_b, zs_b, "-", color="0.5", linewidth=1.2)
axes[0].plot(xs_b[traj.valid][0], zs_b[traj.valid][0], "o", color="black",
             markersize=6, label="start")
last_valid = np.where(traj.valid)[0][-1]
axes[0].plot(xs_b[last_valid], zs_b[last_valid], "s", color="0.3",
             markersize=6, label="end")
for c in closures:
    axes[0].plot([xs_b[c.frame_a], xs_b[c.frame_b]],
                 [zs_b[c.frame_a], zs_b[c.frame_b]],
                 color="tab:red", linewidth=0.8, alpha=0.6, linestyle="--")
axes[0].set_title(f"{DETECTOR.upper()} — bridged (pre-PGO)\n"
                  f"{int(traj.valid.sum())} valid frames")

axes[1].plot(xs_o, zs_o, "-", color="tab:green", linewidth=1.4)
axes[1].plot(xs_o[traj.valid][0], zs_o[traj.valid][0], "o", color="black",
             markersize=6, label="start")
axes[1].plot(xs_o[last_valid], zs_o[last_valid], "s", color="tab:red",
             markersize=6, label="end")
for c in closures:
    axes[1].plot([xs_o[c.frame_a], xs_o[c.frame_b]],
                 [zs_o[c.frame_a], zs_o[c.frame_b]],
                 color="tab:red", linewidth=0.8, alpha=0.6, linestyle="--",
                 label="loop closure" if c is closures[0] else None)
axes[1].set_title(f"{DETECTOR.upper()} — Sim(3) PGO\n"
                  f"chi² {result.initial_chi2:.2e} → {result.final_chi2:.2e}, "
                  f"scale span [{np.nanmin(result.scales[traj.valid]):.2f}, "
                  f"{np.nanmax(result.scales[traj.valid]):.2f}]")

for ax in axes:
    ax.set_aspect("equal")
    ax.set_xlabel("X")
    ax.set_ylabel("Z")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9)

out = TRAJ_DIR / f"{DETECTOR}_pgo_xz.png"
fig.tight_layout()
fig.savefig(out, dpi=130)
plt.close(fig)
print(f"plot -> {out.name}")


# %% [markdown]
# ## Per-node scale plot
#
# 노드별 최적화된 스케일. 1.0 에서 멀어진 정도 = PGO 가 chain 의 scale drift 를
# 얼마나 보정했는지를 직접 보여줌.

# %%
fig, ax = plt.subplots(figsize=(11, 4))
idx = np.arange(len(result.scales))
ax.plot(idx[traj.valid], result.scales[traj.valid], "-", color="tab:blue", linewidth=1.0)
ax.axhline(1.0, color="0.5", linestyle=":", linewidth=0.8)
for c in closures:
    ax.axvspan(c.frame_a - 0.5, c.frame_a + 0.5, color="tab:red", alpha=0.2)
    ax.axvspan(c.frame_b - 0.5, c.frame_b + 0.5, color="tab:red", alpha=0.2)
ax.set_xlabel("frame index")
ax.set_ylabel("optimized scale")
ax.set_title(f"{DETECTOR.upper()} — per-node scale after Sim(3) PGO "
             f"(red bands = loop-closure endpoints)")
ax.grid(alpha=0.3)
out = TRAJ_DIR / f"{DETECTOR}_pgo_scales.png"
fig.tight_layout()
fig.savefig(out, dpi=130)
plt.close(fig)
print(f"plot -> {out.name}")
