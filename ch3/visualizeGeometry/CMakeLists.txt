cmake_minimum_required(VERSION 3.5)
project( visualizeGeometry )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加Eigen头文件
find_package(Eigen3 REQUIRED)

# 添加Pangolin依赖
find_package(Pangolin QUIET)
if(Pangolin_FOUND)
  include_directories(${Pangolin_INCLUDE_DIRS})
  add_executable(visualizeGeometry visualizeGeometry.cpp)
  target_link_libraries(visualizeGeometry ${Pangolin_LIBRARIES})
else()
  message(STATUS "Pangolin not found, skipping visualizeGeometry")
endif()
