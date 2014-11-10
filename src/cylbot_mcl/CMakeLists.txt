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
  cylbot_map_creator
  pcl_ros
  octomap_msgs
  octomap_ros
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

add_library(cylbot_mcl src/pose_cloud.cpp src/pose_cloud2d.cpp src/pose_cloud3d.cpp)
target_link_libraries(cylbot_mcl ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_LIBRARIES})

add_executable(localization_2d src/localization_2d.cpp)
target_link_libraries(localization_2d cylbot_mcl)

add_executable(localization_3d src/localization_3d.cpp)
target_link_libraries(localization_3d cylbot_mcl)

#############
## Testing ##
#############

catkin_add_gtest(pose_cloud_tests test/pose_cloud_tests.cpp)
target_link_libraries(pose_cloud_tests cylbot_mcl)

catkin_add_gtest(transform_tests test/transform_tests.cpp)
target_link_libraries(transform_tests ${catkin_LIBRARIES})