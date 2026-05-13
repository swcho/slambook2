"""matplotlib helpers for the step notebooks.

Each step script imports a couple of these to render an inline figure in
Jupyter. When the same script runs in CI / pytest / plain ``python`` the
helpers still produce a Figure but ``plt.show`` is suppressed by the Agg
backend selection at import time.

The helpers are intentionally thin — they return the ``Figure`` so a notebook
can keep customizing (titles, savefig, etc.) without touching the script flow.
"""

from __future__ import annotations

import os
from typing import Sequence

import matplotlib

# Force a non-GUI backend when running under pytest / no DISPLAY. Notebooks
# override the backend before importing this module (the IPython kernel sets
# MPLBACKEND first), so this only kicks in for the script-mode CI path.
if (
    os.environ.get("MPLBACKEND") is None
    and not os.environ.get("DISPLAY")
    and (os.environ.get("PYTEST_CURRENT_TEST") or os.environ.get("CI"))
):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .dataset import StereoFrame  # noqa: E402


# ---- Step 01 / Step 03 / Step 10 ----------------------------------------


def draw_frame_pair(frame: StereoFrame, title: str | None = None) -> plt.Figure:
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    axes[0].imshow(frame.left, cmap="gray", vmin=0, vmax=255)
    axes[0].set_title(f"frame {frame.index} — left")
    axes[0].axis("off")
    axes[1].imshow(frame.right, cmap="gray", vmin=0, vmax=255)
    axes[1].set_title(f"frame {frame.index} — right")
    axes[1].axis("off")
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    return fig


def draw_keypoints(
    img: np.ndarray,
    pts: np.ndarray,
    *,
    ax: plt.Axes | None = None,
    title: str | None = None,
    color: str = "lime",
    marker_size: float = 8,
) -> plt.Axes:
    """Overlay ``pts`` of shape (N, 2) or (N, 3) on a grayscale image."""
    if ax is None:
        _, ax = plt.subplots(figsize=(10, 4))
    ax.imshow(img, cmap="gray", vmin=0, vmax=255)
    if pts.size > 0:
        ax.scatter(pts[:, 0], pts[:, 1], s=marker_size, edgecolors=color, facecolors="none", linewidths=1)
    if title:
        ax.set_title(title)
    ax.axis("off")
    return ax


def draw_detectors_grid(img: np.ndarray, detector_results: dict[str, np.ndarray]) -> plt.Figure:
    """Show one detector per subplot (Step 03 — 4 detector comparison)."""
    n = len(detector_results)
    fig, axes = plt.subplots(n, 1, figsize=(12, 3 * n), squeeze=False)
    for ax, (name, pts) in zip(axes[:, 0], detector_results.items()):
        draw_keypoints(img, pts, ax=ax, title=f"{name} — {pts.shape[0]} kps", color="lime")
    fig.tight_layout()
    return fig


def draw_mask_overlay(img: np.ndarray, mask: np.ndarray, *, title: str | None = None) -> plt.Figure:
    """Show the image with the excluded mask region tinted red (Step 03 / Step 10)."""
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.imshow(img, cmap="gray", vmin=0, vmax=255)
    overlay = np.zeros((*mask.shape, 4))
    overlay[..., 0] = 1.0
    overlay[..., 3] = (mask == 0).astype(float) * 0.35
    ax.imshow(overlay)
    if title:
        ax.set_title(title)
    ax.axis("off")
    return fig


# ---- Step 04 / Step 07 — flow fields -------------------------------------


def draw_flow_field(
    img: np.ndarray,
    p0: np.ndarray,
    p1: np.ndarray,
    status: np.ndarray | None = None,
    *,
    title: str | None = None,
    step: int = 1,
) -> plt.Figure:
    """Plot arrows from ``p0`` to ``p1`` on top of ``img``.

    ``status`` (length N, ``>0.5`` = tracked) colors the arrows: green for
    tracked, red for lost.
    """
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.imshow(img, cmap="gray", vmin=0, vmax=255)
    p0 = p0[::step]
    p1 = p1[::step]
    if status is not None:
        status = status[::step]
        ok = status > 0.5
        if ok.any():
            ax.quiver(p0[ok, 0], p0[ok, 1], (p1[ok, 0] - p0[ok, 0]), (p1[ok, 1] - p0[ok, 1]),
                      angles="xy", scale_units="xy", scale=1, color="lime", width=0.002, headwidth=4)
        if (~ok).any():
            ax.scatter(p0[~ok, 0], p0[~ok, 1], s=12, c="red", marker="x")
    else:
        ax.quiver(p0[:, 0], p0[:, 1], (p1[:, 0] - p0[:, 0]), (p1[:, 1] - p0[:, 1]),
                  angles="xy", scale_units="xy", scale=1, color="lime", width=0.002, headwidth=4)
    if title:
        ax.set_title(title)
    ax.axis("off")
    return fig


# ---- Step 05 / Step 06 / Step 10 — 3D point cloud ------------------------


def draw_pointcloud_3d(
    pts3d: np.ndarray,
    *,
    title: str | None = None,
    extra_clouds: dict[str, np.ndarray] | None = None,
    elev: float = 18,
    azim: float = -70,
) -> plt.Figure:
    """Scatter a (N, 3) point cloud (camera +Z forward, +Y down)."""
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(pts3d[:, 0], pts3d[:, 1], pts3d[:, 2], s=8, c=pts3d[:, 2], cmap="viridis", label=f"{pts3d.shape[0]} landmarks")
    if extra_clouds:
        for name, cloud in extra_clouds.items():
            ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=14, marker="^", label=name)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z (depth)")
    ax.view_init(elev=elev, azim=azim)
    ax.invert_yaxis()  # camera y points down
    if title:
        ax.set_title(title)
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    return fig


# ---- Step 05 — quality histogram -----------------------------------------


def draw_quality_histogram(
    ratios: np.ndarray,
    *,
    threshold: float | None = None,
    title: str | None = None,
) -> plt.Figure:
    fig, ax = plt.subplots(figsize=(8, 3))
    finite = ratios[np.isfinite(ratios)]
    if finite.size > 0:
        ax.hist(np.log10(np.clip(finite, 1e-20, None)), bins=40, color="steelblue", alpha=0.8)
    if threshold is not None:
        ax.axvline(np.log10(threshold), color="red", linestyle="--", label=f"thr = {threshold:.0e}")
        ax.legend()
    ax.set_xlabel("log10(σ4 / σ3)")
    ax.set_ylabel("count")
    if title:
        ax.set_title(title)
    fig.tight_layout()
    return fig


# ---- Step 08 — PnP residual ---------------------------------------------


def draw_residual_histogram(
    residual_sq: np.ndarray,
    *,
    chi2_threshold: float = 5.991,
    title: str | None = None,
) -> plt.Figure:
    fig, ax = plt.subplots(figsize=(8, 3))
    ax.hist(np.clip(residual_sq, 0, np.percentile(residual_sq, 99) + 1), bins=40, color="steelblue", alpha=0.8)
    ax.axvline(chi2_threshold, color="red", linestyle="--", label=f"chi² thr = {chi2_threshold:.3f}")
    ax.set_xlabel("|z − ẑ|²  (px²)")
    ax.set_ylabel("count")
    ax.legend()
    if title:
        ax.set_title(title)
    fig.tight_layout()
    return fig


# ---- Step 09 / Step 12 — keyframe/window timeline ------------------------


def draw_keyframe_timeline(
    frames: Sequence[int],
    inserts: Sequence[bool],
    *,
    metric: Sequence[float] | None = None,
    title: str | None = None,
) -> plt.Figure:
    fig, ax = plt.subplots(figsize=(8, 3))
    ax.scatter(frames, [0] * len(frames), s=60, c=["red" if b else "lightgray" for b in inserts], edgecolors="black")
    for f, ins in zip(frames, inserts):
        ax.text(f, 0.1, "KF" if ins else "", ha="center", fontsize=8)
    if metric is not None:
        ax2 = ax.twinx()
        ax2.plot(frames, metric, color="steelblue", marker="o", linewidth=1, label="metric")
        ax2.set_ylabel("policy metric")
    ax.set_xlabel("frame index")
    ax.set_yticks([])
    ax.set_ylim(-0.5, 0.5)
    if title:
        ax.set_title(title)
    fig.tight_layout()
    return fig


# ---- Step 11 — BA chi² convergence --------------------------------------


def draw_chi2_per_edge(
    chi2_initial: np.ndarray,
    chi2_final: np.ndarray,
    *,
    title: str | None = None,
) -> plt.Figure:
    fig, axes = plt.subplots(1, 2, figsize=(11, 3))
    for ax, data, name in [(axes[0], chi2_initial, "initial"), (axes[1], chi2_final, "final")]:
        ax.hist(np.log10(np.clip(data, 1e-30, None)), bins=40, color="steelblue", alpha=0.8)
        ax.set_xlabel("log10 per-edge chi²")
        ax.set_title(f"{name}  Σ = {data.sum():.2e}")
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    return fig


# ---- Step 12 — eviction events ------------------------------------------


def draw_eviction_events(
    insert_steps: Sequence[int],
    evicted_kf_ids: Sequence[int | None],
    *,
    title: str | None = None,
) -> plt.Figure:
    fig, ax = plt.subplots(figsize=(8, 3))
    for step, ev in zip(insert_steps, evicted_kf_ids):
        if ev is not None:
            ax.plot([step, step], [0, 1], color="red", linewidth=1)
            ax.text(step, 1.05, f"evict KF#{ev}", rotation=45, fontsize=7, ha="left")
        ax.scatter(step, 0, s=40, c="black")
    ax.set_xlabel("insertion step")
    ax.set_ylim(-0.1, 1.5)
    ax.set_yticks([])
    if title:
        ax.set_title(title)
    fig.tight_layout()
    return fig


# ---- Step 13 — final trajectory + landmark cloud ------------------------


def draw_trajectory_3d(
    poses_T_cw: Sequence[np.ndarray],
    landmarks: np.ndarray | None = None,
    *,
    title: str | None = None,
) -> plt.Figure:
    """Plot a 3D trajectory (camera positions = -R^T t for each T_cw) + landmark cloud."""
    centers = []
    for T in poses_T_cw:
        T = np.asarray(T, dtype=np.float64).reshape(4, 4)
        R = T[:3, :3]; t = T[:3, 3]
        c = -R.T @ t
        centers.append(c)
    centers = np.asarray(centers)
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    if landmarks is not None and landmarks.size > 0:
        ax.scatter(landmarks[:, 0], landmarks[:, 1], landmarks[:, 2], s=4, c="lightgray", alpha=0.6, label="landmarks")
    ax.plot(centers[:, 0], centers[:, 1], centers[:, 2], color="red", marker="o", label="trajectory")
    ax.scatter(centers[0, 0], centers[0, 1], centers[0, 2], s=80, c="green", label="start")
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.invert_yaxis()
    ax.legend(loc="upper left", fontsize=8)
    if title:
        ax.set_title(title)
    fig.tight_layout()
    return fig
