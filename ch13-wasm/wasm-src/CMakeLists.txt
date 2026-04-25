cmake_minimum_required(VERSION 3.20)
project(ch13_wasm CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CH13_WASM_VARIANT selects the compile flavour:
#   baseline | simd | mt | mt-simd
if(NOT DEFINED CH13_WASM_VARIANT)
  set(CH13_WASM_VARIANT "baseline")
endif()

# CH13_WASM_TARGET selects which target to build.
#   hello      (Phase A)         — minimal hello_world binding
#   pnp_spike  (Phase A+ Stage 1) — g2o + Eigen PnP spike
#   ba_spike   (Phase A+ Stage 2) — g2o + Eigen BA spike
#   camera     (Phase B Step 2)   — minimal pinhole Camera binding
if(NOT DEFINED CH13_WASM_TARGET)
  set(CH13_WASM_TARGET "hello")
endif()

set(VARIANT_FLAGS "")
set(VARIANT_LINK "")

if(CH13_WASM_VARIANT MATCHES "simd")
  list(APPEND VARIANT_FLAGS "-msimd128")
endif()

if(CH13_WASM_VARIANT MATCHES "mt")
  list(APPEND VARIANT_FLAGS "-pthread")
  list(APPEND VARIANT_LINK "-pthread" "-sUSE_PTHREADS=1" "-sPTHREAD_POOL_SIZE=4")
endif()

# -----------------------------------------------------------------------------
# Target: hello_world (Phase A)
# -----------------------------------------------------------------------------
if(CH13_WASM_TARGET STREQUAL "hello")
  add_executable(myslam_hello
    myslam/bindings/bind_hello.cpp
  )

  target_compile_options(myslam_hello PRIVATE
    -O3
    ${VARIANT_FLAGS}
  )

  target_link_options(myslam_hello PRIVATE
    -O3
    --bind
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sEXPORT_NAME=createHelloModule
    -sALLOW_MEMORY_GROWTH=1
    -sENVIRONMENT=web,worker,node
    -sWASM=1
    ${VARIANT_LINK}
  )

  set_target_properties(myslam_hello PROPERTIES
    OUTPUT_NAME "myslam_hello.${CH13_WASM_VARIANT}"
    SUFFIX ".js"
  )
endif()

# -----------------------------------------------------------------------------
# Target: camera (Phase B / Step 2) — header-only pinhole binding, no Eigen.
# -----------------------------------------------------------------------------
if(CH13_WASM_TARGET STREQUAL "camera")
  add_executable(myslam_camera
    myslam/bindings/bind_camera.cpp
  )

  target_compile_options(myslam_camera PRIVATE
    -O3
    ${VARIANT_FLAGS}
  )

  target_link_options(myslam_camera PRIVATE
    -O3
    --bind
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sEXPORT_NAME=createCameraModule
    -sALLOW_MEMORY_GROWTH=1
    -sENVIRONMENT=web,worker,node
    -sWASM=1
    ${VARIANT_LINK}
  )

  set_target_properties(myslam_camera PROPERTIES
    OUTPUT_NAME "myslam_camera.${CH13_WASM_VARIANT}"
    SUFFIX ".js"
  )
endif()

# -----------------------------------------------------------------------------
# Targets: pnp_spike (Stage 1) and ba_spike (Stage 2) — Phase A+ gate.
# Share the same g2o setup; only the binding cpp and link-library list differ.
# -----------------------------------------------------------------------------
if(CH13_WASM_TARGET STREQUAL "pnp_spike" OR CH13_WASM_TARGET STREQUAL "ba_spike")
  # ch13-wasm-owned submodule (modern g2o master) — intentionally decoupled
  # from the book's pinned 3rdparty/g2o (2018) to stay compatible with a
  # modern Eigen + CMake.
  set(G2O_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/g2o")
  set(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen")

  # Eigen: vendored as a submodule under third_party/eigen (pinned tag 5.0.1).
  # Using add_subdirectory gives us a fully portable build that does not
  # depend on the host having Homebrew / apt Eigen installed. Modern g2o
  # calls find_package(Eigen3 REQUIRED NO_MODULE) at its top-level CMake,
  # so we enable EIGEN_BUILD_CMAKE_PACKAGE=ON to make Eigen generate an
  # Eigen3Config.cmake into its build dir and then point Eigen3_DIR at it.
  if(NOT EXISTS "${EIGEN_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR
      "Eigen submodule not initialized at ${EIGEN_ROOT}. "
      "Run: git submodule update --init --recursive")
  endif()

  # Quiet Eigen's top-level auxiliaries — we only want the header-only target.
  set(EIGEN_BUILD_CMAKE_PACKAGE ON  CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_DOC           OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_DEMOS         OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_TESTING       OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_BLAS          OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_LAPACK        OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_BTL           OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_SPBENCH       OFF CACHE BOOL "" FORCE)
  set(EIGEN_BUILD_PKGCONFIG     OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING             OFF CACHE BOOL "" FORCE)
  # Keep Eigen from polluting the user's ~/.cmake/packages registry.
  set(CMAKE_EXPORT_NO_PACKAGE_REGISTRY ON)

  add_subdirectory("${EIGEN_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/eigen_build" EXCLUDE_FROM_ALL)
  set(Eigen3_DIR "${CMAKE_CURRENT_BINARY_DIR}/eigen_build" CACHE PATH "" FORCE)
  message(STATUS "[ch13-wasm] Eigen3_DIR=${Eigen3_DIR} (vendored submodule)")

  # g2o: disable everything we don't need for the spike.
  set(G2O_USE_OPENGL OFF CACHE BOOL "" FORCE)
  set(G2O_USE_OPENMP OFF CACHE BOOL "" FORCE)
  set(G2O_USE_CHOLMOD OFF CACHE BOOL "" FORCE)
  set(G2O_USE_CSPARSE OFF CACHE BOOL "" FORCE)
  set(G2O_USE_LOGGING OFF CACHE BOOL "" FORCE)
  set(G2O_USE_LGPL_LIBS OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_APPS OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_SLAM2D_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_SLAM2D_ADDON_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_DATA_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_INSTALL_CMAKE_CONFIG OFF CACHE BOOL "" FORCE)
  set(BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  set(BUILD_LGPL_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  set(G2O_FAST_MATH OFF CACHE BOOL "" FORCE)
  # g2o auto-detects host SSE/AVX and forcibly adds -msseN — incompatible with wasm.
  # On macOS autodetect already falls back to the DISABLE_* path (no /proc/cpuinfo),
  # but those default to OFF and still add the flags, so we pin each to ON.
  set(DO_SSE_AUTODETECT OFF CACHE BOOL "" FORCE)
  set(DISABLE_SSE2   ON CACHE BOOL "" FORCE)
  set(DISABLE_SSE3   ON CACHE BOOL "" FORCE)
  set(DISABLE_SSE4_1 ON CACHE BOOL "" FORCE)
  set(DISABLE_SSE4_2 ON CACHE BOOL "" FORCE)
  set(DISABLE_SSE4_A ON CACHE BOOL "" FORCE)

  # Emscripten needs explicit exception support for g2o (it throws).
  add_compile_options(-fexceptions)
  add_link_options(-fexceptions -sDISABLE_EXCEPTION_CATCHING=0)

  add_subdirectory("${G2O_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/g2o_build" EXCLUDE_FROM_ALL)

  # g2o's CMakeLists pins CMAKE_CXX_STANDARD 11, but Eigen (3.4+/5.x) uses
  # std::enable_if_t (C++14). Override per target to match our C++17 baseline.
  foreach(_tgt core stuff solver_dense solver_eigen types_slam3d types_sba opengl_helper ext_csparse)
    if(TARGET ${_tgt})
      set_target_properties(${_tgt} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
    endif()
  endforeach()

  # Common include dirs for both spikes.
  set(_spike_includes
    "${G2O_ROOT}"
    "${CMAKE_CURRENT_BINARY_DIR}/g2o_build"  # for g2o/config.h
  )

  function(_add_spike_target _name _src _exportName _libs)
    add_executable(${_name} ${_src})
    target_compile_options(${_name} PRIVATE -O3 ${VARIANT_FLAGS})
    target_link_options(${_name} PRIVATE
      -O3 --bind
      -sMODULARIZE=1
      -sEXPORT_ES6=1
      -sEXPORT_NAME=${_exportName}
      -sALLOW_MEMORY_GROWTH=1
      -sENVIRONMENT=web,worker,node
      -sWASM=1
      ${VARIANT_LINK}
    )
    target_include_directories(${_name} PRIVATE ${_spike_includes})
    target_link_libraries(${_name} PRIVATE ${_libs})
    set_target_properties(${_name} PROPERTIES
      OUTPUT_NAME "${_name}.${CH13_WASM_VARIANT}"
      SUFFIX ".js"
    )
  endfunction()

  if(CH13_WASM_TARGET STREQUAL "pnp_spike")
    _add_spike_target(myslam_pnp_spike
      spike/bind_pnp_spike.cpp
      createPnpSpikeModule
      "core;stuff;solver_dense")
  endif()

  if(CH13_WASM_TARGET STREQUAL "ba_spike")
    _add_spike_target(myslam_ba_spike
      spike/bind_ba_spike.cpp
      createBaSpikeModule
      "core;stuff;solver_eigen")
  endif()
endif()

# -----------------------------------------------------------------------------
# Target: cv_spike (Phase C gate) — minimal OpenCV smoke test.
#
# Consumes the pre-built static OpenCV (BUILD_LIST=core,imgproc,features2d,video)
# produced by `bash wasm-src/scripts/build-opencv.sh ${CH13_WASM_VARIANT}`.
# This is the B′ path: OpenCV is a separate sub-build that emits .a libs into
# build/opencv-install-${variant}/, and this top-level CMake imports them
# via find_package(OpenCV ... NO_DEFAULT_PATH). Once the spike passes, the
# real Step 3/4 binding (bind_features.cpp) reuses the same find_package +
# target_link_libraries pattern.
# -----------------------------------------------------------------------------
if(CH13_WASM_TARGET STREQUAL "cv_spike")
  set(_opencv_install
    "${CMAKE_CURRENT_SOURCE_DIR}/build/opencv-install-${CH13_WASM_VARIANT}")
  if(NOT EXISTS "${_opencv_install}/lib/cmake/opencv4/OpenCVConfig.cmake")
    message(FATAL_ERROR
      "OpenCV install not found at ${_opencv_install}. "
      "Run: bash wasm-src/scripts/build-opencv.sh ${CH13_WASM_VARIANT}")
  endif()

  # OpenCV ships its CMake package as <prefix>/lib/cmake/opencv4/, but
  # find_package's case-insensitive heuristic doesn't reliably match the
  # `opencv4` dir to the package name `OpenCV` across CMake versions, so
  # point OpenCV_DIR at the config dir directly.
  set(OpenCV_DIR "${_opencv_install}/lib/cmake/opencv4" CACHE PATH "" FORCE)
  find_package(OpenCV 4.13 REQUIRED
    COMPONENTS core imgproc features2d video
    NO_DEFAULT_PATH)
  message(STATUS "[ch13-wasm] OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")

  add_executable(myslam_cv_spike spike/bind_cv_spike.cpp)
  target_compile_options(myslam_cv_spike PRIVATE -O3 ${VARIANT_FLAGS})
  # OpenCV throws cv::Exception, so we need exception support like the g2o
  # spikes. Keep this scoped to this target so other targets stay lean.
  target_compile_options(myslam_cv_spike PRIVATE -fexceptions)
  target_link_options(myslam_cv_spike PRIVATE
    -O3 --bind
    -fexceptions
    -sDISABLE_EXCEPTION_CATCHING=0
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sEXPORT_NAME=createCvSpikeModule
    -sALLOW_MEMORY_GROWTH=1
    -sENVIRONMENT=web,worker,node
    -sWASM=1
    ${VARIANT_LINK}
  )
  target_link_libraries(myslam_cv_spike PRIVATE ${OpenCV_LIBS})

  set_target_properties(myslam_cv_spike PROPERTIES
    OUTPUT_NAME "myslam_cv_spike.${CH13_WASM_VARIANT}"
    SUFFIX ".js"
  )
endif()
