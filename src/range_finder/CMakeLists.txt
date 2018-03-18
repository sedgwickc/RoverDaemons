cmake_minimum_required(VERSION 2.8.3)
project(range_finder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${range_finder_INCLUDE_DIRS}
)
add_executable(range_finder src/range_finder.cpp)
target_link_libraries(range_finder roboticscape ${catkin_LIBRARIES})

