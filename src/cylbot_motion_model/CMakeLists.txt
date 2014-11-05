cmake_minimum_required(VERSION 2.8.3)
project(cylbot_motion_model)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  cmake_modules
)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

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
#  LIBRARIES cylbot_motion_model
#  CATKIN_DEPENDS geometry_msgs roscpp tf
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
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
)

add_executable(pose_array_test src/pose_array_test.cpp)
target_link_libraries(pose_array_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(motion_model_test src/motion_model_test.cpp)
target_link_libraries(motion_model_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES})