cmake_minimum_required(VERSION 3.0.2)
project(pcd_publisher)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs
        std_msgs
        message_filters
        tf
        pcl_ros
        pcl_conversions
        cv_bridge
        camera_model
)

find_package(PCL 1.7 REQUIRED REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/pcd_publisher_node.cpp src/parameters.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Opencv_LIBRARIES})

add_executable(bw_${PROJECT_NAME}_node src/bw_publisher_node.cpp src/parameters.cpp)
target_link_libraries(bw_${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Opencv_LIBRARIES})

add_executable(my_static_tf2_broadcaster src/my_static_tf2_broadcaster.cpp src/parameters.cpp)
target_link_libraries(my_static_tf2_broadcaster  ${catkin_LIBRARIES} )