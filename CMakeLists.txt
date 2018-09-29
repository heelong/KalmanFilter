cmake_minimum_required(VERSION 3.5)
project(kf)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp 
kf.cpp kf.h 
kf_Fusion.cpp kf_Fusion.h 
ukf.cpp ukf.h 
ekf.cpp ekf.h 
ekf_ctrv.cpp ekf_ctrv.h
ekf_ctrv_geo.cpp ekf_ctrv_geo.h
measurement_package.h ground_truth_package.h)
add_executable(kf ${SOURCE_FILES})