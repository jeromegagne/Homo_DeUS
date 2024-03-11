cmake_minimum_required(VERSION 3.0.2)
project(zTests)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  aruco_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  joint_state_publisher
  moveit_planners_ompl
  moveit_ros_move_group
  moveit_ros_visualization
  moveit_simple_controller_manager
  play_motion
  robot_state_publisher
  xacro
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS joint_state_publisher moveit_planners_ompl moveit_ros_move_group moveit_ros_visualization moveit_simple_controller_manager robot_state_publisher roscpp xacro
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/ArmInterface.cpp)

add_executable(arm_interface_node
  src/homodeus_arm_interface_node.cpp
  src/ArmInterface.cpp
)
target_link_libraries(arm_interface_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


