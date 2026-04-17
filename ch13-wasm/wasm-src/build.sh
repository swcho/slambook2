#!/usr/bin/env bash
# Build a single WASM variant. Usage: bash wasm-src/build.sh [baseline|simd|mt|mt-simd]
set -euo pipefail

VARIANT="${1:-baseline}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

BUILD_DIR="$SCRIPT_DIR/build/$VARIANT"
OUT_DIR="$ROOT_DIR/public/wasm"

# Homebrew's /opt/homebrew/bin/emcc wrapper sets PYTHON but the inner
# /opt/homebrew/Cellar/emscripten/*/libexec/emcc script reads EMSDK_PYTHON
# (falling back to `python3` on PATH — which is macOS 3.9.6 and too old).
# So we set EMSDK_PYTHON ourselves when a newer python is available.
if [ -z "${EMSDK_PYTHON:-}" ]; then
  for candidate in \
    /opt/homebrew/opt/python@3.14/bin/python3.14 \
    /opt/homebrew/opt/python@3.13/bin/python3.13 \
    /opt/homebrew/opt/python@3.12/bin/python3.12 \
    /opt/homebrew/opt/python@3.11/bin/python3.11; do
    if [ -x "$candidate" ]; then
      export EMSDK_PYTHON="$candidate"
      break
    fi
  done
fi

if ! command -v emcmake >/dev/null 2>&1; then
  echo "error: emcmake not found on PATH. Install emscripten (brew install emscripten) or source emsdk_env.sh." >&2
  exit 1
fi

echo "[build.sh] variant=$VARIANT build=$BUILD_DIR out=$OUT_DIR"
mkdir -p "$BUILD_DIR" "$OUT_DIR"

emcmake cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" \
  -DCH13_WASM_VARIANT="$VARIANT" \
  -DCMAKE_BUILD_TYPE=Release

cmake --build "$BUILD_DIR" --parallel

# Copy the produced .js + .wasm (and .worker.js for mt variants) into public/wasm.
for ext in js wasm worker.js; do
  src="$BUILD_DIR/myslam_hello.$VARIANT.$ext"
  if [ -f "$src" ]; then
    cp "$src" "$OUT_DIR/"
    echo "  -> $OUT_DIR/myslam_hello.$VARIANT.$ext"
  fi
done

echo "[build.sh] done."
