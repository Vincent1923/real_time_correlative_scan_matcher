cmake_minimum_required(VERSION 2.8.3)

project(real_time_correlative_scan_matcher)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
"/usr/include/eigen3"
)

add_library(${PROJECT_NAME} SHARED
src/correlative_scan_matcher_2d.cc
src/real_time_correlative_scan_matcher_2d.cc
)
