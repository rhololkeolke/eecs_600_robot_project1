cmake_minimum_required(VERSION 2.8.3)
project(gazebo_custom_multisense_ros_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  sensor_msgs
  std_msgs
  geometry_msgs
  std_srvs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_custom_multisense_ros_plugin
CATKIN_DEPENDS gazebo_plugins
#  DEPENDS system_lib
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  /usr/include/sdformat-2.2
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(HelloWorldPlugin src/hello_world.cpp)
target_link_libraries(HelloWorldPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(ModelPushPlugin src/model_push.cpp)
target_link_libraries(ModelPushPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(CustomMultiSenseSLPlugin src/MultiSenseSLPlugin.cpp)
target_link_libraries(CustomMultiSenseSLPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############
install(TARGETS
  HelloWorldPlugin
  ModelPushPlugin
  CustomMultiSenseSLPlugin
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/plugins/
)
