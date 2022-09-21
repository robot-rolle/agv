cmake_minimum_required(VERSION 3.0.2)
project(hg_robot_agv)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  nav_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  serial
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hg_robot_agv
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation nav_msgs nav_msgs roscpp roslib rospy sensor_msgs serial std_msgs tf
#  DEPENDS system_lib
)

include_directories(include/${PROJECT_NAME})
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(hg_robot_node src/hg_robot.cpp src/hg_robot_data.cpp)
target_link_libraries(hg_robot_node ${catkin_LIBRARIES})


