cmake_minimum_required(VERSION 2.8.3)
project(squirrel_robot_driver)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(catkin REQUIRED
        roscpp
        ros_control
        geometry_msgs
        std_msgs)


catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp ros_control geometry_msgs std_msgs
        DEPENDS dynamixel_lib rt
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
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
        src/dynamixel_sdk/PortHandlerLinux.cpp
)


add_executable(robot_controller_node src/robot.cpp)
add_executable(robot_controller_node src/position_controller.cpp)

add_dependencies(squirrel_robot_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} dynamixel_lib)
add_dependencies(position_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} squirrel_robot_driver)
add_dependencies(robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} squirrel_robot_driver)

target_link_libraries(squirrel_robot_driver
        ${catkin_LIBRARIES}
        dynamixel_lib
        rt
)

target_link_libraries(robot
        ${catkin_LIBRARIES}
        squirrel_robot_driver
)

target_link_libraries(position_controller
        ${catkin_LIBRARIES}
        squirrel_robot_driver
)

FILE(GLOB_RECURSE LibFiles "*.h*")
add_custom_target(headers SOURCES ${LibFiles})

install(DIRECTORY include/dynamixel_sdk
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".git" EXCLUDE
)

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