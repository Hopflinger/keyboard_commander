cmake_minimum_required(VERSION 2.8.3)
project(keyboard_commander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS src include
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(keyboard_commander src/keyboard_commander.cpp)
target_link_libraries(keyboard_commander ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
