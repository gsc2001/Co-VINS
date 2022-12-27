cmake_minimum_required(VERSION 2.8.3)
project(pcd_publisher)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    sensor_msgs
    message_filters
    pcl_ros
    pcl_conversions
    std_msgs
    nav_msgs
    camera_model
    cv_bridge
    roslib
    agent_msg
    tf
    visualization_msgs
    )

find_package(OpenCV)


find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

catkin_package()

add_executable(pcd_publisher
    src/pcd_publisher.cpp
    src/parameters.cpp
    )

target_link_libraries(pcd_publisher ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${CERES_LIBRARIES}) 
message("catkin_lib  ${catkin_LIBRARIES}")
