cmake_minimum_required(VERSION 3.0.2)
project(point_cloud)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_LIBRARIES}
)

link_directories(${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})
add_executable(point_cloud src/main.cpp src/common.cpp src/GuassianProcess.cpp src/CVC_cluster.cpp)

target_link_libraries(point_cloud ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})