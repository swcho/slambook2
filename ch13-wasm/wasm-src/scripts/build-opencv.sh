#!/usr/bin/env bash
# Build OpenCV 4.13.0 as static libs (.a) for Emscripten / WASM.
# Output goes to wasm-src/build/opencv-install/ which our top-level CMakeLists
# then consumes via find_package(OpenCV ... PATHS .../opencv-install NO_DEFAULT_PATH).
#
# Usage:
#   bash wasm-src/scripts/build-opencv.sh [variant]
# variant ∈ {baseline, simd, mt, mt-simd}.  Today only `baseline` is supported
# in the spike — SIMD/MT are added later (Phase C+).
set -euo pipefail

VARIANT="${1:-baseline}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WASM_SRC="$(cd "$SCRIPT_DIR/.." && pwd)"
OPENCV_SRC="$WASM_SRC/third_party/opencv"
BUILD_DIR="$WASM_SRC/build/opencv-build-${VARIANT}"
INSTALL_DIR="$WASM_SRC/build/opencv-install-${VARIANT}"

# Same EMSDK_PYTHON workaround as build.sh — Homebrew's emcc reads
# EMSDK_PYTHON, falling back to a too-old system python3 if unset.
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
  echo "error: emcmake not found on PATH" >&2
  exit 1
fi

if [ ! -f "$OPENCV_SRC/CMakeLists.txt" ]; then
  echo "error: OpenCV submodule not initialized at $OPENCV_SRC" >&2
  echo "       run: git submodule update --init ch13-wasm/wasm-src/third_party/opencv" >&2
  exit 1
fi

EXTRA_FLAGS=""
case "$VARIANT" in
  baseline) ;;
  simd)     EXTRA_FLAGS="-msimd128" ;;
  mt|mt-simd)
    echo "error: variant '$VARIANT' not yet supported in OpenCV spike build" >&2
    exit 2 ;;
  *) echo "unknown variant: $VARIANT" >&2; exit 1 ;;
esac

echo "[build-opencv] variant=$VARIANT"
echo "[build-opencv] src=$OPENCV_SRC"
echo "[build-opencv] build=$BUILD_DIR"
echo "[build-opencv] install=$INSTALL_DIR"

mkdir -p "$BUILD_DIR" "$INSTALL_DIR"

# ----------------------------------------------------------------------------
# Configure flags rationale:
#   BUILD_LIST cherry-picks the four modules we need for ch13 (Step 3 GFTT/FAST
#   detection, Step 4 LK tracking).  Everything else (highgui, dnn, gapi …) is
#   excluded so we don't drag in JPEG/PNG decoders, OpenCL stubs, protobuf,
#   etc.  PNG decoding is done by the browser via createImageBitmap and the
#   raw RGBA pixels cross into WASM through a typed array — OpenCV never sees
#   an encoded image.
#
#   CV_DISABLE_OPTIMIZATION=ON + CPU_BASELINE='' + CPU_DISPATCH='' kill the
#   x86/ARM SIMD intrinsic dispatch tables.  emcc rejects -msseN/-mfpu= flags.
#
#   ENABLE_PIC=FALSE matches platforms/js/build_js.py — wasm has no PIC.
#
#   BUILD_ZLIB=ON: zlib has to be built from source for emscripten.  imgproc
#   doesn't need zlib at runtime but the OpenCV CMake graph still wants it.
# ----------------------------------------------------------------------------

emcmake cmake -S "$OPENCV_SRC" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DBUILD_LIST="core,imgproc,features2d,video" \
  -DBUILD_SHARED_LIBS=OFF \
  -DBUILD_opencv_js=OFF \
  -DBUILD_opencv_apps=OFF \
  -DBUILD_opencv_python2=OFF \
  -DBUILD_opencv_python3=OFF \
  -DBUILD_opencv_python_bindings_generator=OFF \
  -DBUILD_opencv_python_tests=OFF \
  -DBUILD_opencv_java=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PACKAGE=OFF \
  -DBUILD_PROTOBUF=OFF \
  -DBUILD_ZLIB=ON \
  -DBUILD_JPEG=OFF \
  -DBUILD_PNG=OFF \
  -DBUILD_TIFF=OFF \
  -DBUILD_OPENJPEG=OFF \
  -DBUILD_OPENEXR=OFF \
  -DBUILD_WEBP=OFF \
  -DBUILD_TBB=OFF \
  -DBUILD_IPP_IW=OFF \
  -DBUILD_ITT=OFF \
  -DBUILD_JASPER=OFF \
  -DCV_DISABLE_OPTIMIZATION=ON \
  -DCV_ENABLE_INTRINSICS=OFF \
  -DCV_TRACE=OFF \
  -DENABLE_PIC=FALSE \
  -DCPU_BASELINE='' \
  -DCPU_DISPATCH='' \
  -DWITH_1394=OFF \
  -DWITH_ADE=OFF \
  -DWITH_AVIF=OFF \
  -DWITH_EIGEN=OFF \
  -DWITH_FFMPEG=OFF \
  -DWITH_FREETYPE=OFF \
  -DWITH_GPHOTO2=OFF \
  -DWITH_GSTREAMER=OFF \
  -DWITH_GTK=OFF \
  -DWITH_GTK_2_X=OFF \
  -DWITH_IPP=OFF \
  -DWITH_ITT=OFF \
  -DWITH_JASPER=OFF \
  -DWITH_JPEG=OFF \
  -DWITH_LAPACK=OFF \
  -DWITH_OBSENSOR=OFF \
  -DWITH_OPENCL=OFF \
  -DWITH_OPENCLAMDBLAS=OFF \
  -DWITH_OPENCLAMDFFT=OFF \
  -DWITH_OPENCL_SVM=OFF \
  -DWITH_OPENEXR=OFF \
  -DWITH_OPENGL=OFF \
  -DWITH_OPENJPEG=OFF \
  -DWITH_OPENMP=OFF \
  -DWITH_OPENNI=OFF \
  -DWITH_OPENNI2=OFF \
  -DWITH_OPENVX=OFF \
  -DWITH_PNG=OFF \
  -DWITH_PROTOBUF=OFF \
  -DWITH_PTHREADS_PF=OFF \
  -DWITH_QUIRC=OFF \
  -DWITH_TBB=OFF \
  -DWITH_TIFF=OFF \
  -DWITH_V4L=OFF \
  -DWITH_VTK=OFF \
  -DWITH_VULKAN=OFF \
  -DWITH_WEBNN=OFF \
  -DWITH_WEBP=OFF \
  -DCMAKE_C_FLAGS="$EXTRA_FLAGS" \
  -DCMAKE_CXX_FLAGS="$EXTRA_FLAGS"

# Use --parallel so cmake picks the appropriate -j from CMAKE_BUILD_PARALLEL_LEVEL
# or the system default. OpenCV configure already touches everything; our build
# will only rebuild changed targets on subsequent runs.
cmake --build "$BUILD_DIR" --parallel

# Install headers + .a into INSTALL_DIR.
cmake --install "$BUILD_DIR"

echo "[build-opencv] done."
echo "[build-opencv] install layout:"
( cd "$INSTALL_DIR" && find . -maxdepth 3 -type d | sort )
echo "[build-opencv] static libs:"
find "$INSTALL_DIR" -name "libopencv_*.a" | sort
