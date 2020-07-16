cmake_minimum_required(VERSION 3.0)

set(target simple_tracker)

project(${target})
find_package( OpenCV REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(. )

add_executable( ${target} main.cpp )
target_link_libraries( ${target} ${OpenCV_LIBS} m )
target_link_libraries (${target} Eigen3::Eigen)

target_compile_features(${target} PUBLIC cxx_std_20)