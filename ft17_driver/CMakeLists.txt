cmake_minimum_required(VERSION 2.8.3)
project(ft17_driver)

find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS common/include
    LIBRARIES FT17
)

include_directories(common/include)

message(STATUS "Copying ${PROJECT_SOURCE_DIR}/common/lib/libFT17_driver.so to ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libFT17_driver.so")

add_custom_target(
    build_ft17 ALL
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/common/lib/libFT17_driver.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_library(FT17 SHARED)
set_target_properties(FT17 PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(FT17 build_ft17)

install(TARGETS FT17
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS common/lib/libFT17_driver.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY common/include/FT17/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY common doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
