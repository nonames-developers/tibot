cmake_minimum_required(VERSION 3.0.2)
project(tibot_navigation_system)

catkin_python_setup()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
  smach_ros
)

## For test ##
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
# Message1.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MoveFixedPosMsg.srv
  AutomaticExploration.srv
  VoiceMsg.srv
  FigureMsg.srv
)

add_action_files(
  DIRECTORY action
  FILES
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES
  CATKIN_DEPENDS 
  rospy
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
  message_runtime
)

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(tibot_navigation_system
  src/cpp/costmap_client.cpp
  src/cpp/explore.cpp
  src/cpp/frontier_search.cpp
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_dependencies(tibot_navigation_system ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tibot_navigation_system ${catkin_LIBRARIES})


#############
## Install ##
#############

# install nodes
install(TARGETS tibot_navigation_system
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

catkin_add_nosetests(test)









