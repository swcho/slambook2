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
#   hello  (Phase A)  — minimal hello_world binding
#   pnp_spike (Phase A+ Stage 1) — g2o + Eigen PnP spike
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
# Targets: pnp_spike (Stage 1) and ba_spike (Stage 2) — Phase A+ gate.
# Share the same g2o setup; only the binding cpp and link-library list differ.
# -----------------------------------------------------------------------------
if(CH13_WASM_TARGET STREQUAL "pnp_spike" OR CH13_WASM_TARGET STREQUAL "ba_spike")
  # ch13-wasm-owned submodule (modern g2o master) — intentionally decoupled
  # from the book's pinned 3rdparty/g2o (2018) to stay compatible with a
  # modern Eigen + CMake.
  set(G2O_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/g2o")

  # Eigen: header-only, but modern g2o uses find_package(Eigen3 NO_MODULE)
  # which needs Eigen3Config.cmake. Point at the Homebrew cellar cmake dir.
  if(NOT DEFINED Eigen3_DIR)
    foreach(_candidate
        /opt/homebrew/share/eigen3/cmake
        /opt/homebrew/opt/eigen/share/eigen3/cmake
        /usr/local/share/eigen3/cmake)
      if(EXISTS "${_candidate}/Eigen3Config.cmake")
        set(Eigen3_DIR "${_candidate}" CACHE PATH "" FORCE)
        break()
      endif()
    endforeach()
    # Fallback: glob the cellar for any Eigen version.
    if(NOT Eigen3_DIR)
      file(GLOB _eigen_cellar_cmake "/opt/homebrew/Cellar/eigen/*/share/eigen3/cmake")
      if(_eigen_cellar_cmake)
        list(GET _eigen_cellar_cmake 0 Eigen3_DIR)
        set(Eigen3_DIR "${Eigen3_DIR}" CACHE PATH "" FORCE)
      endif()
    endif()
  endif()
  if(NOT Eigen3_DIR)
    message(FATAL_ERROR "Eigen3_DIR not set — install via 'brew install eigen' or set Eigen3_DIR manually")
  endif()
  message(STATUS "[ch13-wasm] Eigen3_DIR=${Eigen3_DIR}")

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

  # g2o's CMakeLists pins CMAKE_CXX_STANDARD 11, but Homebrew Eigen 3.4 uses
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
