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
  message_generation
  )

add_message_files(DIRECTORY msg FILES

  Diagnostics.msg
  )

generate_dynamic_reconfigure_options(
  cfg/multirotor_simulator.cfg
  )

generate_messages(DEPENDENCIES

  std_msgs
  geometry_msgs
  sensor_msgs
  )

set(LIBRARIES
  MrsMultirotorSimulator
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  INCLUDE_DIRS include include/controllers
  CATKIN_DEPENDS std_msgs mrs_msgs mrs_lib roscpp std_srvs mrs_uav_hw_api nav_msgs dynamic_reconfigure rosgraph_msgs message_runtime
  )

include_directories(
  include
  include/controllers
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# Multirotor simulator

add_library(MrsMultirotorSimulator
  src/multirotor_simulator.cpp
  src/quadrotor_model.cpp

  src/controllers/mixer.cpp
  src/controllers/rate_controller.cpp
  src/controllers/attitude_controller.cpp
  )

add_dependencies(MrsMultirotorSimulator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsMultirotorSimulator
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# Hw API Plugin

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

install(DIRECTORY launch config msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
