cmake_minimum_required(VERSION 2.8.3)
project(cylbot_description)


find_package(catkin REQUIRED roscpp sensor_msgs tf laser_geometry)

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
#  LIBRARIES cylbot_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hokuyo_laser_to_pointcloud src/hokuyo_laser_to_pointcloud.cpp)
target_link_libraries(hokuyo_laser_to_pointcloud ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS
  hokuyo_laser_to_pointcloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)