cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(camera_fusion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4.1 REQUIRED)

add_library(fusion_core
    src/camFusion_Student.cpp
    src/matching2D_Student.cpp
)
target_include_directories(fusion_core PUBLIC src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fusion_core PUBLIC ${OpenCV_LIBRARIES})
target_compile_options(fusion_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(3D_object_tracking
    src/FinalProject_Camera.cpp
    src/lidarData.cpp
    src/objectDetection2D.cpp
)
target_include_directories(3D_object_tracking PRIVATE src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(3D_object_tracking PRIVATE fusion_core ${OpenCV_LIBRARIES})
target_compile_options(3D_object_tracking PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(fusion_tests tests/fusion_tests.cpp)
target_include_directories(fusion_tests PRIVATE src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fusion_tests PRIVATE fusion_core ${OpenCV_LIBRARIES})
target_compile_options(fusion_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME fusion_tests COMMAND fusion_tests)
