cmake_minimum_required(VERSION 2.8.3)
project(kclhand_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_srvs
  std_msgs
  tf
  message_generation
  actionlib_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
    srv
  FILES
    handConfiguration.srv
    getCurrentConfiguration.srv
    HandOperationMode.srv
    MoveHand.srv
    MoveFinger.srv
)

## Generate action stuff
add_action_files(
  DIRECTORY action
  FILES OpenFingers.action ActuateHand.action MoveFinger.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
    std_srvs
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ftd2xx
    EposCmd
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    roscpp
    rospy
    sensor_msgs
    tf
    actionlib_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_custom_target(build_EposCmd
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/lib/squirrel_kclhand/libEposCmd.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
	COMMENT "copy EposCmd"
)

add_library(EposCmd SHARED)
set_target_properties(EposCmd PROPERTIES LINKER_LANGUAGE CXX) 
add_dependencies(EposCmd build_EposCmd)

add_custom_target(build_ftd2xx
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/lib/squirrel_kclhand/libftd2xx.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
	COMMENT "Copying ${PROJECT_SOURCE_DIR}/lib/squirrel_kclhand/libftd2xx.so to ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}"
)

add_library(ftd2xx SHARED)
set_target_properties(ftd2xx PROPERTIES LINKER_LANGUAGE CXX) 
add_dependencies(ftd2xx build_ftd2xx)

# add_executable(state_publisher src/state_publisher.cpp)
# target_link_libraries(state_publisher ${catkin_LIBRARIES})
# 
# include_directories(include ${catkin_INCLUDE_DIRS})
# 
# add_executable(metaHand src/metaHand.cpp)
# target_link_libraries(metaHand ${catkin_LIBRARIES} EposCmd)
# 
# add_executable(metahand_control src/metahand_control.cpp)
# target_link_libraries(metahand_control ${catkin_LIBRARIES} EposCmd)
# 
# add_executable(controller_metahand src/metaHand_controller.cpp)
# target_link_libraries(controller_metahand ${catkin_LIBRARIES} EposCmd)
# 
# add_executable(wien_controller_metahand src/Metahand_vienna_controller.cpp)
# target_link_libraries(wien_controller_metahand ${catkin_LIBRARIES} EposCmd)
# 
# add_executable(controller_metahand_v2 src/Metahand_controller_v2.cpp)
# target_link_libraries(controller_metahand_v2 ${catkin_LIBRARIES} EposCmd)
# 
# add_executable(squirrel_kclhand_controller src/squirrel_kclhand_controller.cpp)
# target_link_libraries(squirrel_kclhand_controller ${catkin_LIBRARIES} EposCmd)
# 
# add_executable(metahand_controller_N1 src/metahand_controller_N1.cpp)
# target_link_libraries(metahand_controller_N1 ${catkin_LIBRARIES} EposCmd)
# 
# add_executable(metahand_controller_N1_review src/metahand_controller_N1_review.cpp)
# target_link_libraries(metahand_controller_N1_review ${catkin_LIBRARIES} EposCmd)
# 
# add_executable(palm_stability src/palm_stability.cpp)
# target_link_libraries(palm_stability ${catkin_LIBRARIES} EposCmd)
# 
# add_executable(controller_metahand_review src/metahand_controller_cali_new.cpp)
# target_link_libraries(controller_metahand_review ${catkin_LIBRARIES} EposCmd)

# add_executable(kclhand_controller src/kclhand_controller_impedance.cpp)
# target_link_libraries(kclhand_controller ${catkin_LIBRARIES} ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libEposCmd.so)
# add_dependencies(kclhand_controller kclhand_control_generate_messages_cpp EposCmd)

add_executable(kclhand_controller src/kclhand_controller.cpp)
target_link_libraries(kclhand_controller ${catkin_LIBRARIES} ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libEposCmd.so)
add_dependencies(kclhand_controller kclhand_control_generate_messages_cpp EposCmd)



### INSTALL ###
install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES lib/squirrel_kclhand/libftd2xx.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES lib/squirrel_kclhand/libEposCmd.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
