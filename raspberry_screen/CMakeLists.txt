cmake_minimum_required(VERSION 2.8.3)
project(raspberry_screen)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

#catkin_python_setup()


## Generate services in the 'srv' folder
 add_service_files(
   FILES
   DisplayScreen.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime)




#############
## Install ##
#############

install(PROGRAMS
  src/raspberry_screen_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
