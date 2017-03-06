cmake_minimum_required(VERSION 2.8.3)
project(wrist_safety)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs 
)

find_package(Boost REQUIRED COMPONENTS
filesystem
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wrist_safety src/wrist_safety.cpp)
target_link_libraries(wrist_safety ${catkin_LIBRARIES})
