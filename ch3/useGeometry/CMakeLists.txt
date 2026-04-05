cmake_minimum_required(VERSION 3.5)
project( geometry )

# 添加Eigen头文件
find_package(Eigen3 REQUIRED)

add_executable(eigenGeometry useGeometry.cpp)
target_link_libraries(eigenGeometry Eigen3::Eigen)
