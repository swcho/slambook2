"""matplotlib helpers for the step notebooks.

Kept minimal — each step only needs a couple of plots and we don't want a heavy
visualization API. When called in a non-interactive (CI/headless) context we
suppress ``plt.show`` so the same script doubles as a pytest fixture.
"""

from __future__ import annotations

import os

import matplotlib

# Force a non-GUI backend when running under pytest / no DISPLAY. Notebooks
# override the backend before importing this module, so this only kicks in for
# the script-mode CI path.
if os.environ.get("MPLBACKEND") is None and not os.environ.get("DISPLAY") and os.environ.get("PYTEST_CURRENT_TEST"):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .dataset import StereoFrame  # noqa: E402


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
