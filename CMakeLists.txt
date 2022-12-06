cmake_minimum_required(VERSION 2.8.3)
project(keyboard_manual)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(keyboard_manual src/keyboard_manual.cpp)
target_link_libraries(keyboard_manual ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
