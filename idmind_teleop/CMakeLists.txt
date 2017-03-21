cmake_minimum_required(VERSION 2.8.3)
project(idmind_teleop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs)

catkin_package()

#############
## Install ##
#############

install(PROGRAMS src/motors_api.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS src/squirrel_control.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS src/teleop_control.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
