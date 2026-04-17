cmake_minimum_required(VERSION 3.20)
project(ch13_wasm CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CH13_WASM_VARIANT selects the compile flavour:
#   baseline | simd | mt | mt-simd
if(NOT DEFINED CH13_WASM_VARIANT)
  set(CH13_WASM_VARIANT "baseline")
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

# Phase A: hello_world module only. Real myslam sources join in Phase B+.
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
  -sENVIRONMENT=web,worker
  -sWASM=1
  ${VARIANT_LINK}
)

set_target_properties(myslam_hello PROPERTIES
  OUTPUT_NAME "myslam_hello.${CH13_WASM_VARIANT}"
  SUFFIX ".js"
)
