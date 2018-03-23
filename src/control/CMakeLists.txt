cmake_minimum_required(VERSION 2.8.3)
project(control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control
#  CATKIN_DEPENDS actionlib_msgs geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${drive_INCLUDE_DIRS}
)
add_executable(control src/control.cpp)
target_link_libraries(control roboticscape ${catkin_LIBRARIES})
