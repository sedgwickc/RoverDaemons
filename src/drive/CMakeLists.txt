cmake_minimum_required(VERSION 2.8.3)
project(drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs
    )
# Build MotorControl library
#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#  ${drive_INCLUDE_DIRS}
#)
#add_library(MotorControl src/MotorControl.cpp)
#target_link_libraries(MotorControl roboticscape ${catkin_LIBRARIES})

# build ROS drive node
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${drive_INCLUDE_DIRS}
)
add_executable(drive src/drive.cpp src/MotorControl.cpp)
target_link_libraries(drive roboticscape ${catkin_LIBRARIES})

