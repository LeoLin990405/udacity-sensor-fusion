cmake_minimum_required(VERSION 3.10)
project(Localization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fPIC -DNDEBUG -Wextra -O3")

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CARLALIB_DIR ${CMAKE_SOURCE_DIR}/libcarla-install/lib)
set(CARLA_LIBS
    ${CARLALIB_DIR}/libboost_filesystem.a
    ${CARLALIB_DIR}/libboost_program_options.a
    ${CARLALIB_DIR}/libboost_python38.a
    ${CARLALIB_DIR}/libboost_system.a
    ${CARLALIB_DIR}/libcarla_client.a
    ${CARLALIB_DIR}/librpc.a
    ${CARLALIB_DIR}/libDebugUtils.a
    ${CARLALIB_DIR}/libDetour.a
    ${CARLALIB_DIR}/libDetourCrowd.a
    ${CARLALIB_DIR}/libDetourTileCache.a
    ${CARLALIB_DIR}/libRecast.a)

link_directories(libcarla-install/lib)
include_directories(libcarla-install/include)
include_directories(libcarla-install/include/system)
include_directories(rpclib/include)

add_executable(cloud_loc c3-main.cpp helper.cpp scan_matcher.cpp)
target_link_libraries(cloud_loc ${PCL_LIBRARIES} ${CARLA_LIBS})

option(BUILD_SCAN_MATCHER_TESTS "Build the offline scan-matching test" OFF)
if(BUILD_SCAN_MATCHER_TESTS)
  enable_testing()
  add_executable(scan_matcher_test scan_matcher_test.cpp scan_matcher.cpp)
  target_link_libraries(scan_matcher_test ${PCL_LIBRARIES})
  add_test(
      NAME scan_matcher_offline
      COMMAND scan_matcher_test ${CMAKE_SOURCE_DIR}/map.pcd)
endif()
