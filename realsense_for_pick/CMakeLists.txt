cmake_minimum_required(VERSION 2.8.3)
project(realsense_for_pick)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport  
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED)


# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV  REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories ( ${OpenCV_LIB_DIR} )
MESSAGE(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")

include_directories ( ${OpenCV_INCLUDE_DIRS} )
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR}/)
include_directories(${CMAKE_SOURCE_DIR}/src/)


add_executable(pcl_visualize_Realsense_for_pick src/pcl_visualize_Realsense_for_pick.cpp)
target_link_libraries(pcl_visualize_Realsense_for_pick ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(pcl_visualize_Realsense_for_pick1 src/pcl_visualize_Realsense_for_pick1.cpp)
target_link_libraries(pcl_visualize_Realsense_for_pick1 ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

