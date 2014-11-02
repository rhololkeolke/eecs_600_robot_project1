cmake_minimum_required(VERSION 2.8.3)
project(multisense_sensor_model)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  pcl_conversions
  pcl_ros
  gazebo_ros
  geometry_msgs
  visualization_msgs
)
find_package(Boost REQUIRED)
find_package(gazebo REQUIRED)

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
  INCLUDE_DIRS include
#  LIBRARIES multisense_sensor_model
#  CATKIN_DEPENDS gazebo_plugins
#  DEPENDS eigen
)

###########
## Build ##
###########

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_executable(hokuyo_beam_params_em src/hokuyo_beam_params_em.cpp)
target_link_libraries(hokuyo_beam_params_em ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(BoxInfoPublisher src/box_info_publisher.cpp)
target_link_libraries(BoxInfoPublisher ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

install(TARGETS
  hokuyo_beam_params_em
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  BoxInfoPublisher
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/plugins/
)
