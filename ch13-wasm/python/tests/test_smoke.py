"""Regression gate for the Python reference (Step 1–13).

Mirrors the key assertions in each step's percent-format script so CI can run
a single ``uv run pytest`` to gate the entire reference. The scripts themselves
are still the primary documentation/exploration surface.
"""

from __future__ import annotations

import cv2
import numpy as np

from myslam_ref.ba import optimize
from myslam_ref.camera import Camera, project_batch, round_trip_max_error
from myslam_ref.dataset import load_kitti_dataset
from myslam_ref.features import Detector, detect, render_synth_frame, track_lk
from myslam_ref.keyframe import decide_fixed_interval, decide_frame_distance, decide_inlier_threshold
from myslam_ref.pnp import estimate_pose
from myslam_ref.se3 import se3_from_translation, se3_log_norm
from myslam_ref.slam_map import Policy, PolicyOptions, SlamMap, build_synthetic_kf_stream
from myslam_ref.triangulation import Algo, triangulate


# ---- Step 01 --------------------------------------------------------------


def test_kitti_mini_loads_5_stereo_frames():
    ds = load_kitti_dataset()
    assert len(ds.cameras) == 4
    assert len(ds.frames) == 5
    for f in ds.frames:
        assert f.left.shape == (370, 1226)
        assert f.right.shape == (370, 1226)
        assert f.left.dtype.name == "uint8"


def test_kitti_mini_camera_intrinsics_and_baseline():
    ds = load_kitti_dataset()
    cam0 = ds.camera0
    cam1 = ds.camera1
    assert cam0.fx == 707.0912
    assert cam0.fy == 707.0912
    assert cam0.cx == 601.8873
    assert cam0.cy == 183.1104
    assert abs(cam0.baseline_m) < 1e-9
    assert abs(cam1.baseline_m - 0.5371657) < 1e-4
    assert abs(cam1.t[0] + 0.5371657) < 1e-4
    assert abs(cam1.t[1]) < 1e-9
    assert abs(cam1.t[2]) < 1e-9


# ---- Step 02 --------------------------------------------------------------


def test_camera_round_trip_machine_precision():
    cam = Camera(fx=520, fy=520, cx=320, cy=240, baseline=0)
    rng = np.random.default_rng(0)
    pts = rng.normal(scale=2.0, size=(1000, 3))
    pts[:, 2] = np.abs(pts[:, 2]) + 1.0
    assert round_trip_max_error(cam, pts) < 1e-10


def test_camera_stereo_disparity_matches_baseline_over_depth():
    ds = load_kitti_dataset()
    cam0 = Camera(fx=ds.camera0.fx, fy=ds.camera0.fy,
                  cx=ds.camera0.cx, cy=ds.camera0.cy, baseline=0.0)
    cam1 = Camera(
        fx=ds.camera1.fx, fy=ds.camera1.fy, cx=ds.camera1.cx, cy=ds.camera1.cy,
        baseline=ds.camera1.baseline_m, tx=ds.camera1.t[0], ty=ds.camera1.t[1], tz=ds.camera1.t[2],
    )
    Z = 10.0
    uv_l = cam0.world_to_pixel((0.0, 0.0, Z))
    uv_r = cam1.world_to_pixel((0.0, 0.0, Z))
    observed = uv_l[0] - uv_r[0]
    predicted = cam0.fx * cam1.baseline / Z
    assert abs(observed - predicted) < 1e-6


def test_project_batch_shape_and_consistency():
    cam = Camera(fx=520, fy=520, cx=320, cy=240, baseline=0)
    pts = np.array([[1.0, -0.5, 5.0], [0.0, 0.0, 7.0]])
    uv = project_batch(cam, pts)
    assert uv.shape == (2, 2)
    assert np.allclose(uv[0], cam.camera_to_pixel(pts[0]))


# ---- Step 03 --------------------------------------------------------------


def test_synth_frame_4_detectors_above_threshold():
    frame0 = render_synth_frame(0, 0)
    for algo in [Detector.GFTT, Detector.HARRIS, Detector.FAST, Detector.ORB]:
        pts = detect(frame0, algo, maxFeatures=200,
                     qualityLevel=0.01, minDistance=20, blockSize=3)
        assert pts.shape[0] >= 30, f"{algo.name}: expected ≥ 30 kps, got {pts.shape[0]}"


def test_gftt_mask_excludes_hole():
    frame0 = render_synth_frame(0, 0)
    h, w = frame0.shape
    cx_hole, cy_hole = 588, 170
    mask = np.full((h, w), 255, dtype=np.uint8)
    mask[max(0, cy_hole - 50):cy_hole + 50,
         max(0, cx_hole - 50):cx_hole + 50] = 0
    pts = detect(frame0, Detector.GFTT, maxFeatures=200,
                 qualityLevel=0.01, minDistance=20, mask=mask)
    in_hole = int(
        np.sum(
            (pts[:, 0] >= cx_hole - 50) & (pts[:, 0] < cx_hole + 50)
            & (pts[:, 1] >= cy_hole - 50) & (pts[:, 1] < cy_hole + 50)
        )
    )
    assert in_hole == 0, f"mask broken: {in_hole} keypoints inside hole"


def test_lk_left_right_synth_disparity_minus_22():
    left = render_synth_frame(0, 0)
    right = render_synth_frame(0, 1)
    seeds = detect(left, Detector.GFTT, maxFeatures=200,
                   qualityLevel=0.01, minDistance=20)
    out = track_lk(left, right, seeds, winSize=11,
                   maxLevel=3, maxIter=30, eps=0.01)
    ok = out[:, 2] > 0.5
    rate = float(ok.mean())
    mean_dx = float((out[ok, 0] - seeds[ok, 0]).mean())
    mean_dy = float((out[ok, 1] - seeds[ok, 1]).mean())
    assert rate >= 0.7
    assert abs(mean_dx + 22) < 3
    assert abs(mean_dy) < 1


def test_kitti_real_frame_gftt_returns_kps():
    ds = load_kitti_dataset()
    pts = detect(ds.frames[0].left, Detector.GFTT,
                 maxFeatures=200, qualityLevel=0.01, minDistance=20)
    assert pts.shape[0] >= 50


# ---- Step 04 --------------------------------------------------------------


def test_stereo_lk_rectified_pair_dy_near_zero():
    ds = load_kitti_dataset()
    seeds = detect(ds.frames[0].left, Detector.GFTT,
                   maxFeatures=200, qualityLevel=0.01, minDistance=20)
    tracked = track_lk(ds.frames[0].left, ds.frames[0].right, seeds)
    ok = tracked[:, 2] > 0.5
    assert ok.mean() >= 0.7
    dys = tracked[ok, 1] - seeds[ok, 1]
    dxs = tracked[ok, 0] - seeds[ok, 0]
    assert abs(float(dys.mean())) < 1.5  # rectified pair → mean dy ≈ 0
    assert float(dxs.mean()) < 0  # left → right shift is negative


# ---- Step 05 --------------------------------------------------------------


def _stereo_setup_kitti_05_half():
    """KITTI 05 @ 0.5× downsample — same numbers as verify_triangulation.ts."""
    fx, fy, cx, cy = 360.295, 360.295, 303.605, 92.695
    baseline = 0.537151
    k = np.array([fx, fy, cx, cy])
    T_l = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [
                   0, 0, 1, 0]], dtype=np.float64)
    T_r = np.array([[1, 0, 0, -baseline], [0, 1, 0, 0],
                   [0, 0, 1, 0]], dtype=np.float64)
    return fx, fy, cx, cy, baseline, k, T_l, T_r


def _grid_world_points():
    pts = []
    for z_step in range(4):
        z = 5 + z_step * 4
        for x_step in range(-2, 3):
            for y_step in range(-1, 2):
                pts.append([x_step * 1.5, y_step * 0.8, z])
    return np.asarray(pts, dtype=np.float64)


def test_triangulation_linear_svd_exact():
    fx, fy, cx, cy, baseline, k, T_l, T_r = _stereo_setup_kitti_05_half()
    gt = _grid_world_points()
    N = gt.shape[0]
    left_pts = np.zeros((N, 3))
    right_pts = np.zeros((N, 3))
    for i, (x, y, z) in enumerate(gt):
        left_pts[i] = [fx * x / z + cx, fy * y / z + cy, 1.0]
        right_pts[i] = [fx * (x - baseline) / z + cx, fy * y / z + cy, 1.0]
    svd = triangulate(left_pts, right_pts, k, T_l, k, T_r,
                      algo=Algo.LINEAR_SVD, quality_threshold=0.01)
    assert np.linalg.norm(svd[:, :3] - gt, axis=1).max() < 1e-6
    assert svd[:, 3].max() < 1e-9
    assert int((svd[:, 4] > 0.5).sum()) == N


def test_triangulation_lost_status_yields_zero_nan():
    fx, fy, cx, cy, baseline, k, T_l, T_r = _stereo_setup_kitti_05_half()
    gt = _grid_world_points()
    N = gt.shape[0]
    left_pts = np.zeros((N, 3))
    right_pts = np.zeros((N, 3))
    for i, (x, y, z) in enumerate(gt):
        left_pts[i] = [fx * x / z + cx, fy * y / z + cy, 1.0]
        right_pts[i] = [fx * (x - baseline) / z + cx, fy * y / z + cy, 1.0]
    right_pts[0, 2] = 0.0  # lost
    out = triangulate(left_pts, right_pts, k, T_l, k, T_r)
    assert (out[0, :3] == 0).all() and np.isnan(out[0, 3]) and out[0, 4] == 0


def test_triangulation_inverted_return_flips_ok():
    fx, fy, cx, cy, baseline, k, T_l, T_r = _stereo_setup_kitti_05_half()
    gt = _grid_world_points()
    N = gt.shape[0]
    left_pts = np.zeros((N, 3))
    right_pts = np.zeros((N, 3))
    for i, (x, y, z) in enumerate(gt):
        left_pts[i] = [fx * x / z + cx, fy * y / z + cy, 1.0]
        right_pts[i] = [fx * (x - baseline) / z + cx, fy * y / z + cy, 1.0]
    right_pts[:, 1] += 4.0  # epipolar noise
    fixed = triangulate(left_pts, right_pts, k, T_l, k, T_r,
                        quality_threshold=1e-3, inverted_return=False)
    inverted = triangulate(left_pts, right_pts, k, T_l, k,
                           T_r, quality_threshold=1e-3, inverted_return=True)
    assert int(np.sum((fixed[:, 4] > 0.5) == (inverted[:, 4] > 0.5))) == 0


# ---- Step 06 / Step 07 ----------------------------------------------------


def test_initial_map_yields_landmarks_on_kitti_mini():
    ds = load_kitti_dataset()
    cam0, cam1 = ds.camera0, ds.camera1
    frame0 = ds.frames[0]
    seeds = detect(frame0.left, Detector.GFTT, maxFeatures=200,
                   qualityLevel=0.01, minDistance=20)
    tracked = track_lk(frame0.left, frame0.right, seeds)
    k = np.array([cam0.fx, cam0.fy, cam0.cx, cam0.cy])
    T_l = np.hstack([np.eye(3), np.zeros((3, 1))])
    T_r = np.hstack([np.eye(3), np.array([[-cam1.baseline_m], [0], [0]])])
    pts = triangulate(seeds, tracked, k, T_l, k, T_r,
                      algo=Algo.LINEAR_SVD, quality_threshold=0.01)
    accepted = pts[pts[:, 4] > 0.5]
    assert accepted.shape[0] >= 30
    assert accepted[:, 2].min() > 0


def test_temporal_tracking_chain():
    ds = load_kitti_dataset()
    prev = ds.frames[0].left
    seeds_prev = detect(prev, Detector.GFTT, maxFeatures=200,
                        qualityLevel=0.01, minDistance=20)
    for frame in ds.frames[1:]:
        curr = frame.left
        tracked = track_lk(prev, curr, seeds_prev)
        rate = float((tracked[:, 2] > 0.5).mean())
        assert rate >= 0.7
        prev = curr
        seeds_prev = detect(prev, Detector.GFTT, maxFeatures=200,
                            qualityLevel=0.01, minDistance=20)


# ---- Step 08 --------------------------------------------------------------


def _pnp_case(seed: int, N: int, noise: float, outlier_fraction: float = 0.0):
    state = [int(seed) & 0xFFFFFFFF]

    def rand() -> float:
        state[0] = (state[0] + 0x6D2B79F5) & 0xFFFFFFFF
        t = state[0]
        t = ((t ^ (t >> 15)) * (1 | t)) & 0xFFFFFFFF
        t = ((t + ((t ^ (t >> 7)) * (61 | t))) ^ t) & 0xFFFFFFFF
        return float((t ^ (t >> 14)) & 0xFFFFFFFF) / 4294967296.0

    def uni():
        return rand() * 2 - 1

    def gauss():
        u = max(rand(), 1e-12)
        v = rand()
        return float(np.sqrt(-2 * np.log(u)) * np.cos(2 * np.pi * v))

    R_gt = cv2.Rodrigues(
        np.array([uni() * 0.3, uni() * 0.3, uni() * 0.3]).reshape(3, 1))[0]
    t_gt = np.array([uni() * 0.2, uni() * 0.2, 2 + rand() * 0.5])
    K = np.array([[520, 0, 320], [0, 520, 240], [0, 0, 1]], dtype=np.float64)
    pts3, pts2, outliers = [], [], []
    i = 0
    while i < N:
        Pw = np.array([uni() * 1.5, uni() * 1.5, uni() * 1.5])
        Pc = R_gt @ Pw + t_gt
        if Pc[2] <= 0.5:
            continue
        is_out = outlier_fraction > 0 and rand() < outlier_fraction
        nu = gauss() * noise if noise else 0
        nv = gauss() * noise if noise else 0
        u = 520 * Pc[0] / Pc[2] + 320 + nu
        v = 520 * Pc[1] / Pc[2] + 240 + nv
        if is_out:
            u += 60 if uni() >= 0 else -60
            v += 60 if uni() >= 0 else -60
            outliers.append(i)
        pts3.append(Pw)
        pts2.append([u, v])
        i += 1
    rvec_gt = cv2.Rodrigues(R_gt)[0].reshape(-1)
    init6 = np.concatenate([
        t_gt + np.array([uni() * 0.3, uni() * 0.3, uni() * 0.3]),
        rvec_gt + np.array([uni() * 0.1, uni() * 0.1, uni() * 0.1]),
    ])
    return np.asarray(pts3), np.asarray(pts2), K, R_gt, t_gt, init6, outliers


def test_pnp_noiseless_machine_precision():
    for s in range(1, 6):
        pts3, pts2, K, R_gt, t_gt, init6, _ = _pnp_case(s, 40, 0.0)
        res = estimate_pose(pts3, pts2, K, init6)
        rvec, _ = cv2.Rodrigues(res.T_cw[:, :3] @ R_gt.T)
        assert float(np.linalg.norm(rvec)) < 1e-6
        assert float(np.linalg.norm(res.T_cw[:, 3] - t_gt)) < 1e-6


def test_pnp_outlier_recall_above_80pct():
    for s in range(20, 23):
        pts3, pts2, K, R_gt, t_gt, init6, outliers = _pnp_case(
            s, 100, 1.0, 0.2)
        res = estimate_pose(pts3, pts2, K, init6)
        hits = sum(1 for idx in outliers if res.final_inlier_mask[idx] == 0)
        recall = hits / max(1, len(outliers))
        assert recall >= 0.8


# ---- Step 09 --------------------------------------------------------------


def test_keyframe_inlier_threshold():
    assert decide_inlier_threshold(50, min_inliers=80).insert
    assert not decide_inlier_threshold(120, min_inliers=80).insert


def test_keyframe_frame_distance_and_interval():
    p0 = se3_from_translation([0, 0, 0])
    p1 = se3_from_translation([0, 0, 0.6])
    assert decide_frame_distance(p1, p0, min_distance_m=0.5).insert
    assert not decide_fixed_interval(3, 0, interval=5).insert
    assert decide_fixed_interval(6, 0, interval=5).insert


# ---- Step 11 --------------------------------------------------------------


def test_ba_noiseless_converges():
    # Smaller scene than the verify suite — pytest budget.
    P, L = 3, 12
    fx = fy = 520
    cx = 240
    cy = 200
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    rng = np.random.default_rng(seed=42)
    poses_gt = [(np.eye(3), np.zeros(3))]
    for _ in range(1, P):
        rvec = rng.uniform(-0.2, 0.2, 3)
        R, _ = cv2.Rodrigues(rvec)
        t = rng.uniform(-0.3, 0.3, 3)
        poses_gt.append((R, t))
    lms_gt = np.column_stack(
        [rng.uniform(-1, 1, L), rng.uniform(-1, 1, L), rng.uniform(2, 4, L)])
    obs = []
    for p in range(P):
        R, t = poses_gt[p]
        for li in range(L):
            Pc = R @ lms_gt[li] + t
            if Pc[2] <= 0.5:
                continue
            obs.append([p, li, fx * Pc[0] / Pc[2] +
                       cx, fy * Pc[1] / Pc[2] + cy, 1])
    init_poses = np.zeros((P, 12))
    for p in range(P):
        R, t = poses_gt[p]
        pert = 0 if p == 0 else 0.1
        rvec = cv2.Rodrigues(R)[0].reshape(-1) + pert * rng.uniform(-1, 1, 3)
        tinit = t + pert * rng.uniform(-1, 1, 3)
        R_init = cv2.Rodrigues(rvec)[0]
        T = np.hstack([R_init, tinit.reshape(3, 1)])
        init_poses[p] = T.reshape(-1)
    init_lms = lms_gt + 0.1 * rng.uniform(-1, 1, (L, 3))
    IDENT = np.hstack([np.eye(3), np.zeros((3, 1))])
    res = optimize(init_poses, init_lms, np.asarray(obs),
                   np.array([0]), K, IDENT, IDENT,
                   iterations=20, chi2_init=5.991, adaptive_rounds=5)
    assert res.final_chi2_sum < 1e-8


# ---- Step 12 --------------------------------------------------------------


def test_slam_map_fifo_evicts_chronologically():
    stream = build_synthetic_kf_stream(5, 2, 3)
    smap = SlamMap(num_active_keyframes=4)
    for lp in stream["landmark_positions"]:
        smap.insert_map_point(lp)
    evictions = []
    for i, pose in enumerate(stream["poses"]):
        _, ev = smap.insert_keyframe(
            i, pose, stream["observed_landmark_ids"][i], "fifo", PolicyOptions(0.2))
        if ev is not None:
            evictions.append(ev.evicted_kf_id)
    assert evictions == [0, 1, 2, 3, 4, 5]
    assert sorted(smap.active_keyframe_ids) == [6, 7, 8, 9]


def test_slam_map_clean_drops_orphan_landmarks():
    smap = SlamMap(num_active_keyframes=1)
    lm_a = smap.insert_map_point([0, 0, 5])
    lm_b = smap.insert_map_point([1, 0, 5])
    smap.insert_keyframe(0, np.eye(4), {lm_a}, "fifo", PolicyOptions(0.2))
    T = np.eye(4)
    T[2, 3] = 0.5
    smap.insert_keyframe(1, T, {lm_b}, "fifo", PolicyOptions(0.2))
    assert lm_a not in smap.active_landmark_ids
    assert lm_b in smap.active_landmark_ids


def test_se3_log_norm_translation():
    T = se3_from_translation([0.5, 0, 0])
    assert abs(se3_log_norm(T) - 0.5) < 1e-9


# ---- Step 13 --------------------------------------------------------------


def test_full_pipeline_runs_on_kitti_mini():
    ds = load_kitti_dataset()
    cam0, cam1 = ds.camera0, ds.camera1
    K = cam0.K
    T_left = np.hstack([np.eye(3), np.zeros((3, 1))])
    T_right = np.hstack([np.eye(3), np.array([[-cam1.baseline_m], [0], [0]])])
    frame0 = ds.frames[0]
    seeds = detect(frame0.left, Detector.GFTT, maxFeatures=150,
                   qualityLevel=0.01, minDistance=20)
    tracked_right = track_lk(frame0.left, frame0.right, seeds)
    tri = triangulate(seeds, tracked_right,
                      np.array([cam0.fx, cam0.fy, cam0.cx, cam0.cy]), T_left,
                      np.array([cam1.fx, cam1.fy, cam1.cx, cam1.cy]), T_right,
                      algo=Algo.LINEAR_SVD, quality_threshold=0.01)
    valid = tri[:, 4] > 0.5
    lms_world = tri[valid, :3]
    lm_pts = seeds[valid][:, :2]
    assert lms_world.shape[0] >= 30
    trajectory = [np.eye(4)]
    prev_left = frame0.left
    prev_pts = lm_pts
    prev_lms = lms_world
    for frame in ds.frames[1:]:
        seeds_in = np.hstack([prev_pts, np.ones((prev_pts.shape[0], 1))])
        tracked = track_lk(prev_left, frame.left, seeds_in)
        ok = tracked[:, 2] > 0.5
        assert ok.mean() >= 0.5
        init6 = np.concatenate(
            [trajectory[-1][:3, 3], cv2.Rodrigues(trajectory[-1][:3, :3])[0].reshape(-1)])
        res = estimate_pose(prev_lms[ok], tracked[ok, :2], K, init6)
        T_new = np.eye(4)
        T_new[:3, :] = res.T_cw
        trajectory.append(T_new)
        prev_left = frame.left
        prev_pts = tracked[ok, :2]
        prev_lms = prev_lms[ok]
    ts = np.array([T[:3, 3] for T in trajectory])
    assert float(np.linalg.norm(ts[-1] - ts[0])) > 0.1
