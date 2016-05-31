cmake_minimum_required (VERSION 2.6)
project (SIGNED_DISTANCE_FUNCTION)

set(EXECUTABLE_OUTPUT_PATH /home/dede/bin/)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(signed_distance_function main.cpp three_d_grid.cpp)
target_link_libraries(signed_distance_function ${PCL_LIBRARIES} ${VTK_LIBRARIES})

