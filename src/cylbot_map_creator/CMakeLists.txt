cmake_minimum_required(VERSION 2.8.3)
project(cylbot_map_creator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cylbot_map_creator
#  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_map_publisher src/test_map_publisher.cpp)
target_link_libraries(test_map_publisher ${catkin_LIBRARIES})

add_executable(2d_map_creator src/2d_map_creator.cpp)
target_link_libraries(2d_map_creator ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS
  test_map_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)