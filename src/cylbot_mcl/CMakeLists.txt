cmake_minimum_required(VERSION 2.8.3)
project(cylbot_mcl)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cylbot_motion_model
  geometry_msgs
  multisense_sensor_model
  roscpp
  sensor_msgs
  tf
  nav_msgs
  pcl_ros
)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

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
  LIBRARIES cylbot_mcl
#  CATKIN_DEPENDS cmake_modules cylbot_motion_model geometry_msgs multisense_sensor_model roscpp sensor_msgs tf
  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_library(cylbot_mcl src/pose_cloud.cpp)
target_link_libraries(cylbot_mcl ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_LIBRARIES})

add_executable(localization_2d src/localization_2d.cpp)
target_link_libraries(localization_2d cylbot_mcl)

#############
## Testing ##
#############

catkin_add_gtest(pose_cloud_tests test/pose_cloud_tests.cpp)
target_link_libraries(pose_cloud_tests cylbot_mcl)