cmake_minimum_required(VERSION 2.8.3)
project(squirrel_control)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(catkin REQUIRED
        cmake_modules
        hardware_interface
        controller_manager
        roscpp
        control_msgs
        trajectory_msgs
        actionlib
        urdf
        joint_limits_interface
        transmission_interface
        control_toolbox
        std_msgs
        sensor_msgs
        squirrel_safety_msgs
        rosparam_shortcuts)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS
          include
        CATKIN_DEPENDS
          hardware_interface
          controller_manager
          roscpp
          control_msgs
          trajectory_msgs
          urdf
          joint_limits_interface
          transmission_interface
          control_toolbox
          std_msgs
          sensor_msgs
          rosparam_shortcuts
        LIBRARIES
          squirrel_hw_control_loop
          squirrel_hw_interface
          squirrel_sim_hw_interface
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

add_library(dynamixel_lib
        src/dynamixel_sdk/GroupBulkRead.cpp
        src/dynamixel_sdk/GroupBulkWrite.cpp
        src/dynamixel_sdk/GroupSyncRead.cpp
        src/dynamixel_sdk/GroupSyncWrite.cpp
        src/dynamixel_sdk/PacketHandler.cpp
        src/dynamixel_sdk/PortHandler.cpp
        src/dynamixel_sdk/Protocol1PacketHandler.cpp
        src/dynamixel_sdk/Protocol2PacketHandler.cpp
        src/dynamixel_sdk/PortHandlerLinux.cpp)

# Motor utils
add_library(motor_utilities src/motor_utilities.cpp)
target_link_libraries(motor_utilities
        dynamixel_lib
        ${catkin_LIBRARIES}
        )

# Control loop
add_library(squirrel_hw_control_loop
        src/squirrel_hw_control_loop.cpp)
target_link_libraries(squirrel_hw_control_loop
        ${catkin_LIBRARIES}
        )

# Hardware Interface
add_library(squirrel_hw_interface
        src/squirrel_hw_interface.cpp
        )
target_link_libraries(squirrel_hw_interface
        motor_utilities
        ${catkin_LIBRARIES}
        )

# Simulation Hardware Interface
add_library(squirrel_sim_hw_interface
        src/squirrel_sim_hw_interface.cpp
        )
target_link_libraries(squirrel_sim_hw_interface
        squirrel_hw_interface
        ${catkin_LIBRARIES}
        )

# Main control executables
add_executable(${PROJECT_NAME}_sim_hw_main src/squirrel_sim_hw_main.cpp)
set_target_properties(${PROJECT_NAME}_sim_hw_main PROPERTIES OUTPUT_NAME squirrel_sim_hw_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_sim_hw_main
        squirrel_sim_hw_interface
        squirrel_hw_control_loop
        ${catkin_LIBRARIES}
        )

add_executable(${PROJECT_NAME}_hw_main src/squirrel_hw_main.cpp)
set_target_properties(${PROJECT_NAME}_hw_main PROPERTIES OUTPUT_NAME squirrel_hw_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_hw_main
        squirrel_hw_interface
        squirrel_hw_control_loop
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

install(TARGETS
  squirrel_hw_control_loop
  squirrel_hw_interface
  squirrel_sim_hw_interface
  LIBRARY DESTINATION
  ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
  ${PROJECT_NAME}_hw_main
  ${PROJECT_NAME}_sim_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})