cmake_minimum_required(VERSION 3.10)
project(control_trajectory_tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(pid_demo main.cpp pid_controller.cpp)
