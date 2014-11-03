cmake_minimum_required(VERSION 2.8.3)
project(octomap_tests)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap_msgs
)

find_package(octomap REQUIRED)

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
#  LIBRARIES octomap_tests
#  CATKIN_DEPENDS octomap roscpp
  DEPENDS octomap
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(simple_example src/simple_example.cpp)
target_link_libraries(simple_example ${OCTOMAP_LIBRARIES})

add_executable(octree_to_rviz_test src/octree_to_rviz_test.cpp)
target_link_libraries(octree_to_rviz_test ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})