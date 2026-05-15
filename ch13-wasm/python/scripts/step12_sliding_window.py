# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Step 12 — Sliding window map management
#
# `wasm-src/spike/verify_slam.ts` 의 정책별 게이트와 ``cleanMap`` 검증을 Python
# 으로 미러. 4 정책 (`ch13-default` / `fifo` / `covisibility` / `distance-only`)
# 이 모두 합성 KF 스트림에서 의도된 6 회 eviction 을 만들고, FIFO 가 시간 순서를
# 따르며, orphan landmark 가 `cleanMap` 으로 정리됨을 확인한다.

# %%
import numpy as np

from myslam_ref.se3 import se3_from_translation, se3_log_norm
from myslam_ref.slam_map import Policy, PolicyOptions, SlamMap, build_synthetic_kf_stream
from myslam_ref.viz import draw_eviction_events, draw_trajectory_3d  # noqa: F401

# %% [markdown]
# ## 1. se3_log_norm sanity (verify_slam.ts §se3.logNorm)

# %%
I = np.eye(4)
assert abs(se3_log_norm(I)) < 1e-12

T_trans = se3_from_translation([0.5, 0.0, 0.0])
print(f"|log(translation 0.5)| = {se3_log_norm(T_trans):.6f}")
assert abs(se3_log_norm(T_trans) - 0.5) < 1e-9

# 30° yaw + 0.3 m forward
c, s = float(np.cos(np.pi / 6)), float(np.sin(np.pi / 6))
T_yaw = np.array([[c, 0, s, 0.3], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
expected = float(np.hypot(0.3, np.pi / 6))
got = se3_log_norm(T_yaw)
print(f"|log(30° yaw + 0.3 m)| = {got:.6f}, expected {expected:.6f}")
assert abs(got - expected) < 5e-3

# %% [markdown]
# ## 2. Policy iteration on synthetic KF stream

# %%
def run_policy(policy: str, window_size: int = 4):
    stream = build_synthetic_kf_stream(5, 2, 3)
    smap = SlamMap(num_active_keyframes=window_size)
    for lp in stream["landmark_positions"]:
        smap.insert_map_point(lp)
    evictions = []
    for i, pose in enumerate(stream["poses"]):
        _, ev = smap.insert_keyframe(i, pose, stream["observed_landmark_ids"][i], policy, PolicyOptions(0.2))
        if ev is not None:
            evictions.append(ev.evicted_kf_id)
    return evictions, sorted(smap.active_keyframe_ids), len(smap.active_landmark_ids)


# 정책별 결과 — verify_slam.ts §SlamMap eviction policies 의 기대값과 일치해야 한다.
ev_default, active_default, _ = run_policy("ch13-default")
ev_fifo, active_fifo, _ = run_policy("fifo")
ev_cov, active_cov, _ = run_policy("covisibility")
ev_dist, active_dist, _ = run_policy("distance-only")

print(f"ch13-default: 6 evictions, final active size {len(active_default)} — {ev_default}")
print(f"fifo        : evictions {ev_fifo}")
print(f"covisibility: 6 evictions, final active size {len(active_cov)} — {ev_cov}")
print(f"distance-only: 6 evictions — {ev_dist}")

assert len(ev_default) == 6 and len(active_default) == 4
assert ev_fifo == [0, 1, 2, 3, 4, 5], f"FIFO order wrong: {ev_fifo}"
assert sorted(active_fifo) == [6, 7, 8, 9]
assert len(ev_cov) == 6 and len(active_cov) == 4
assert len(ev_dist) == 6

# %% [markdown]
# ## 3. cleanMap drops orphan landmarks

# %%
smap = SlamMap(num_active_keyframes=1)
lm_a = smap.insert_map_point([0, 0, 5])
lm_b = smap.insert_map_point([1, 0, 5])
I4 = np.eye(4)
smap.insert_keyframe(0, I4, {lm_a}, "fifo", PolicyOptions(0.2))
T = np.eye(4); T[2, 3] = 0.5
smap.insert_keyframe(1, T, {lm_b}, "fifo", PolicyOptions(0.2))
print(f"after KF#0 evicted: active landmarks = {sorted(smap.active_landmark_ids)}")
assert lm_a not in smap.active_landmark_ids
assert lm_b in smap.active_landmark_ids

# %% [markdown]
# ## 4. Active pose spread

# %%
stream = build_synthetic_kf_stream(5, 0, 0)
smap = SlamMap(num_active_keyframes=5)
for lp in stream["landmark_positions"]:
    smap.insert_map_point(lp)
for i, pose in enumerate(stream["poses"]):
    smap.insert_keyframe(i, pose, stream["observed_landmark_ids"][i], "fifo", PolicyOptions(0.2))
_mean, variance, n = smap.active_pose_spread()
print(f"5 forward KFs: pose spread variance = {variance:.4f} over {n} KFs")
assert variance > 0 and np.isfinite(variance)

# %% [markdown]
# ## 5. 시각화 — eviction 타임라인 + 합성 KF stream trajectory

# %%
# Re-run ch13-default and capture eviction step by step for the plot.
stream_viz = build_synthetic_kf_stream(5, 2, 3)
smap_viz = SlamMap(num_active_keyframes=4)
for lp in stream_viz["landmark_positions"]:
    smap_viz.insert_map_point(lp)
ev_steps = []
ev_ids: list[int | None] = []
for i, pose in enumerate(stream_viz["poses"]):
    _, ev = smap_viz.insert_keyframe(i, pose, stream_viz["observed_landmark_ids"][i], "ch13-default", PolicyOptions(0.2))
    ev_steps.append(i)
    ev_ids.append(ev.evicted_kf_id if ev else None)
fig_ev = draw_eviction_events(ev_steps, ev_ids, title="ch13-default policy · KF insertion → eviction events")

# %%
fig_tj = draw_trajectory_3d(
    stream_viz["poses"],
    landmarks=stream_viz["landmark_positions"],
    title="synthetic KF stream · trajectory (5 forward + 2 dup + 3 forward)",
)

# %%
print("OK — step12 SlamMap: 4 policies + cleanMap + pose spread all consistent with verify_slam.ts")
