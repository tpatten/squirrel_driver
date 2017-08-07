cmake_minimum_required(VERSION 2.8.3)
project(screen)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_service_files(
   FILES
   DisplayScreen.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )
