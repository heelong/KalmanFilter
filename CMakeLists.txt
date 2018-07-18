cmake_minimum_required(VERSION 3.5)
project(kf)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp kf.cpp kf.h ukf.cpp ukf.h ekf.cpp ekf.h measurement_package.h ground_truth_package.h)
add_executable(kf ${SOURCE_FILES})