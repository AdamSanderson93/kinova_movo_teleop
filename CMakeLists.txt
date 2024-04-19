cmake_minimum_required(VERSION 3.0.2)
project(kinova_movo_teleop)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  movo_7dof_moveit_config
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  roscpp
  roslaunch
  roslint
  tf2
  std_msgs
  geometry_msgs
  trajectory_msgs
  bio_ik
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system chrono)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinova_movo_teleop
  CATKIN_DEPENDS movo_7dof_moveit_config
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_servo
    roscpp
    roslaunch
    roslint
    tf
    tf2
    tf2_ros
    std_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(control_node src/BIO_IK_Group.cpp src/BIO_IK_Planner.cpp src/control_node.cpp)

target_link_libraries(control_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS control_node 
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
