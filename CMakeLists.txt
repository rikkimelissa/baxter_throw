cmake_minimum_required(VERSION 2.8.3)
project(baxter_throw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  baxter_interface
  urdf_parser
  pykdl_utils
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)