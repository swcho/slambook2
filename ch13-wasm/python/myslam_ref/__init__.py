"""Python reference for ch13-wasm visual SLAM pipeline.

Each module mirrors the semantics of a WASM binding (wasm-src/myslam/bindings/*.cpp)
using opencv-python + numpy + scipy, so per-step outputs can be cross-checked
against the WASM/UI without relying on the same code path.
"""

__all__ = ["dataset", "camera", "features", "viz"]
