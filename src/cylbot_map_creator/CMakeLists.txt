cmake_minimum_required(VERSION 2.8.3)
project(cylbot_map_creator)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

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
  octomap_msgs
  octomap_ros
  message_generation
  multisense_sensor_model
)
find_package(octomap REQUIRED)
find_package(YamlCpp REQUIRED)

add_message_files(FILES LikelihoodField.msg)

generate_messages(
  DEPENDENCIES
  nav_msgs
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
  CATKIN_DEPENDS message_runtime
  DEPENDS octomap
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIR}
)

add_executable(test_map_publisher src/test_map_publisher.cpp)
target_link_libraries(test_map_publisher ${catkin_LIBRARIES})

add_executable(2d_map_creator src/2d_map_creator.cpp)
target_link_libraries(2d_map_creator ${catkin_LIBRARIES})

add_executable(octomap_creator src/octomap_creator.cpp)
target_link_libraries(octomap_creator ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(my_octomap_saver src/octomap_saver.cpp)
target_link_libraries(my_octomap_saver ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(likelihood_field_generator src/likelihood_field_generator.cpp)
target_link_libraries(likelihood_field_generator ${catkin_LIBRARIES} ${YAMLCPP_LIBRARY})

add_executable(likelihood_field_server src/likelihood_field_server.cpp)
target_link_libraries(likelihood_field_server ${catkin_LIBRARIES} ${YAMLCPP_LIBRARY})

add_executable(publish_likelihood_field_as_map src/publish_likelihood_field_as_map)
target_link_libraries(publish_likelihood_field_as_map ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS
  test_map_publisher
  2d_map_creator
  octomap_creator
  my_octomap_saver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)