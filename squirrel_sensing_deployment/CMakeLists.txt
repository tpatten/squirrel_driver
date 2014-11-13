cmake_minimum_required(VERSION 2.8.3)
project(squirrel_sensing_deployment)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  	squirrel_sensor_tester
  	squirrel_sensing
  	squirrel_sensing_coordinator
)


catkin_package()
