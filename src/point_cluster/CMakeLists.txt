cmake_minimum_required(VERSION 3.0.2)
project(point_cluster)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        roslaunch
        autoware_msgs
        # cv_bridge    //开启这个会造成opencv的冲突
        pcl_conversions
        pcl_ros
        geometry_msgs
        laser_geometry
        sensor_msgs
        message_generation
        visualization_msgs
        # common_lib
        # object_builders_lib
        # grid_map_ros
        # grid_map_cv               //注释掉避免在xavier编译时与pcl1.8的kdtree_flannch
        # grid_map_msgs
        jsk_rviz_plugins
        jsk_recognition_msgs
)

find_package(OpenCV REQUIRED)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

find_package(PCL 1.8 REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
        # INCLUDE_DIRS
        # LIBRARIES
        # CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ../common_lib/include/
        ../object_builders_lib/include/
        include
)

add_executable(demo src/demo.cpp src/clip.cpp src/CVC/CVC.cpp src/CVC/CVC_cluster.cpp src/detect_object.cpp)
target_link_libraries(demo ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} common_lib object_builders_lib)
# add_dependencies(demo patchworkpp_generate_messages_cpp)

