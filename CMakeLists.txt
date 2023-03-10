cmake_minimum_required(VERSION 3.1.2)
project(mrs_multirotor_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# add_compile_options("-Og")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_msgs
  mrs_lib
  nodelet
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  mrs_uav_hw_api
  rosgraph_msgs
  dynamic_reconfigure
  geometry_msgs
  )

generate_dynamic_reconfigure_options(
  cfg/multirotor_simulator.cfg
  )

set(LIBRARIES
  MrsMultirotorSimulator
  MrsMultirotorSimSystemRos
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  INCLUDE_DIRS include include/mrs_multirotor_simulator include/mrs_multirotor_simulator/uav_system include/mrs_multirotor_simulator/uav_system/controllers
  CATKIN_DEPENDS std_msgs mrs_msgs mrs_lib roscpp std_srvs mrs_uav_hw_api nav_msgs dynamic_reconfigure rosgraph_msgs geometry_msgs
  )

include_directories(
  include
  include/mrs_multirotor_simulator
  include/mrs_multirotor_simulator/uav_system
  include/mrs_multirotor_simulator/uav_system/controllers
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# ROS Wrapper for the Multirotor UAV System + controllers

add_library(MrsMultirotorSimSystemRos
  src/uav_system_ros.cpp
  )

add_dependencies(MrsMultirotorSimSystemRos
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsMultirotorSimSystemRos
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# Multirotor simulator
# - ROS node with the MRS Multirotor Simulator

add_library(MrsMultirotorSimulator
  src/multirotor_simulator.cpp
  )

add_dependencies(MrsMultirotorSimulator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsMultirotorSimulator
  MrsMultirotorSimSystemRos
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# Hw API Plugin
# - pluginlib library for interfacing the Simulator with the MRS UAV System

add_library(MrsSimulatorApiPlugin
  src/hw_api_plugin.cpp
  )

add_dependencies(MrsSimulatorApiPlugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsSimulatorApiPlugin
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config tmux
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
