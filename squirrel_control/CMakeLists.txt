cmake_minimum_required(VERSION 2.8.3)
project(squirrel_control)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(catkin REQUIRED
        roscpp
        hardware_interface
        joint_limits_interface
        controller_manager
        std_msgs
        geometry_msgs
        trajectory_msgs
        squirrel_safety_msgs)

find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
         arm_interface
        CATKIN_DEPENDS
         roscpp
         hardware_interface
         joint_limits_interface
         controller_manager
         std_msgs
         geometry_msgs
         trajectory_msgs
         squirrel_safety_msgs
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

add_library(motor_utils src/motor_utilities.cpp)
target_link_libraries(motor_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES} dynamixel_lib)
add_dependencies(motor_utils ${catkin_EXPORTED_TARGETS})

add_library(arm_interface src/arm_hardware_interface.cpp)
target_link_libraries(arm_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES} motor_utils)
add_dependencies(arm_interface ${catkin_EXPORTED_TARGETS})

add_executable(squirrel_hardware_interface src/squirrel_hardware_interface.cpp)
target_link_libraries(squirrel_hardware_interface
        arm_interface
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )

add_dependencies(squirrel_hardware_interface ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

FILE(GLOB_RECURSE LibFiles "*.h*")
add_custom_target(headers SOURCES ${LibFiles})

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".git" EXCLUDE
)

install(TARGETS
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
