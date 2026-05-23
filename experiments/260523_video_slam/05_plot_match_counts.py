# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
# ---

# %% [markdown]
# # Plot — number of matches between consecutive frames
#
# `04_match_pairwise.py` 가 저장한 `pairwise_matches.npz` 를 detector별로 읽어
# (i, i+1) pair마다 매칭 수를 시각화한다.
#
# - 위 panel : detector 별 곡선 (linear y)
# - 아래 panel: 같은 데이터 (log y) + 매칭 0인 구간을 음영 처리
#
# 출력
# - `data/260523_house/keyframes/matches/match_counts_by_detector.png`

# %%
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from keyframe import KeyframeStore

HERE = Path.cwd() if "__file__" not in globals() else Path(__file__).resolve().parent
DATA_DIR = HERE.parent / "data" / "260523_house"
STORE_DIR = DATA_DIR / "keyframes"
MATCH_DIR = STORE_DIR / "matches"

DETECTORS = ["sift", "orb", "akaze", "brisk"]
COLORS = {"sift": "tab:blue", "orb": "tab:orange", "akaze": "tab:green", "brisk": "tab:red"}


# %% [markdown]
# ## 로드 — 각 detector의 (P,) 매칭 카운트

# %%
store = KeyframeStore.load(STORE_DIR, load_features=False)
n_pairs = len(store.frames) - 1
print(f"frames={len(store.frames)}  pairs={n_pairs}")

counts: dict[str, np.ndarray] = {}
for det in DETECTORS:
    with np.load(MATCH_DIR / det / "pairwise_matches.npz") as z:
        offs = z["offsets"]
    counts[det] = np.diff(offs).astype(np.int32)
    assert len(counts[det]) == n_pairs

# Contiguous runs where ALL detectors are empty — the problem windows.
all_zero = np.logical_and.reduce([counts[d] == 0 for d in DETECTORS])

def runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous True runs as [start, end) intervals."""
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            out.append((i, j))
            i = j
        else:
            i += 1
    return out

zero_runs = runs(all_zero)
print(f"all-detector-zero pairs: {int(all_zero.sum())} in {len(zero_runs)} runs")


# %% [markdown]
# ## Plot

# %%
x = np.arange(n_pairs)
fig, (ax_lin, ax_log) = plt.subplots(2, 1, figsize=(13, 7), sharex=True)

for ax in (ax_lin, ax_log):
    for s, e in zero_runs:
        ax.axvspan(s, e - 1, color="red", alpha=0.10,
                   label="all-detector zero" if (s, e) == zero_runs[0] else None)

for det in DETECTORS:
    ax_lin.plot(x, counts[det], label=det.upper(), color=COLORS[det], linewidth=1)
    # +1 so zeros stay visible at the bottom of the log axis
    ax_log.plot(x, counts[det] + 1, label=det.upper(), color=COLORS[det], linewidth=1)

ax_lin.set_ylabel("good matches (Lowe 0.75)")
ax_lin.set_title("Pairwise matches per detector — frame i ↔ i+1")
ax_lin.grid(alpha=0.3)
ax_lin.legend(loc="upper right", fontsize=9, ncol=5)

ax_log.set_yscale("log")
ax_log.set_xlabel("pair index (frame i -> i+1)")
ax_log.set_ylabel("good matches + 1 (log)")
ax_log.grid(alpha=0.3, which="both")

# Per-detector median annotations on the linear panel.
medians = {d: int(np.median(counts[d])) for d in DETECTORS}
txt = "  ".join(f"{d.upper()} med={medians[d]}" for d in DETECTORS)
ax_lin.text(
    0.01, 0.95, txt, transform=ax_lin.transAxes,
    ha="left", va="top", fontsize=9,
    bbox=dict(facecolor="white", alpha=0.8, edgecolor="0.7"),
)

fig.tight_layout()
out_path = MATCH_DIR / "match_counts_by_detector.png"
fig.savefig(out_path, dpi=130)
plt.show()
print(f"plot -> {out_path}")


# %% [markdown]
# ## 요약 출력

# %%
print("\nthresholded pair counts (≥k matches between i, i+1):")
print(f"  {'k':>5}  " + "  ".join(f"{d.upper():>5}" for d in DETECTORS))
for k in (1, 10, 30, 100, 300):
    line = f"  {k:>5}  " + "  ".join(
        f"{int((counts[d] >= k).sum()):>5d}" for d in DETECTORS
    )
    print(line)

print("\nzero-match runs (all detectors):")
for s, e in zero_runs:
    a = store.frames[s].id
    b = store.frames[e].id
    print(f"  pair {s:>3d}..{e-1:<3d}  ({a} .. {b})  width={e - s}")
