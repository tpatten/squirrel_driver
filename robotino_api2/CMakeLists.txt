cmake_minimum_required(VERSION 2.8.3)
project(robotino_api2)

find_package(catkin REQUIRED)

add_custom_target(install_robotino_api2
	ALL
		${PROJECT_SOURCE_DIR}/install_robotino_api2.bash
)