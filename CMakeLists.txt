cmake_minimum_required(VERSION 2.8.3)
project(pose_game)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# FIND_LIBRARY(EIGEN_LIBRARY Eigen ~/libraries/c++/eigen)
# INCLUDE_DIRECTORIES(~/libraries/c++/eigen)
# LINK_DIRECTORIES(~/libraries/c++/eigen)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_commander
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moveit_vive
#  CATKIN_DEPENDS geometry_msgs moveit_commander moveit_msgs rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
)

###########
## Build ##
###########



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  # ${EIGEN_LIBRARY}
)


# add_executable(vive_moveit_jointConstraints src/vive_moveit_jointConstraints.cpp)
# target_link_libraries(vive_moveit_jointConstraints ${catkin_LIBRARIES})
# add_dependencies(vive_moveit_jointConstraints moveit_vive)


