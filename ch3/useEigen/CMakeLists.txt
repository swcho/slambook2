cmake_minimum_required(VERSION 3.5)
project(useEigen)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

# 添加Eigen头文件
find_package(Eigen3 REQUIRED)
add_executable(eigenMatrix eigenMatrix.cpp)
target_link_libraries(eigenMatrix Eigen3::Eigen)
