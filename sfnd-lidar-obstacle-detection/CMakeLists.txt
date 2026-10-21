cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(lidar_obstacle_detection)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable(environment src/environment.cpp src/render/render.cpp)
target_compile_options(environment PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(environment ${PCL_LIBRARIES} ${Boost_LIBRARIES})

enable_testing()

add_executable(kdtree_clustering_test tests/kdtree_clustering_test.cpp)
target_include_directories(kdtree_clustering_test PRIVATE src)
target_compile_options(kdtree_clustering_test PRIVATE -Wall -Wextra -pedantic)
add_test(NAME kdtree_clustering_test COMMAND kdtree_clustering_test)

add_executable(pipeline_test tests/pipeline_test.cpp)
target_include_directories(pipeline_test PRIVATE src)
target_compile_definitions(
    pipeline_test PRIVATE PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_compile_options(pipeline_test PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(pipeline_test ${PCL_LIBRARIES} ${Boost_LIBRARIES})
add_test(NAME pipeline_test COMMAND pipeline_test)
