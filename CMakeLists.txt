cmake_minimum_required(VERSION 3.5)
project(mrs_multirotor_simulator)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  mrs_msgs
  mrs_lib
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

include_directories(
  include
  include/mrs_multirotor_simulator
  include/mrs_multirotor_simulator/uav_system
  include/mrs_multirotor_simulator/uav_system/controllers
  ${rclcpp_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

set(LIBRARIES
  MrsMultirotorSimulator_Simulator
  MrsMultirotorSimulator_SimulatorSystemRos
  # MrsMultirotorSimulator_HwApiPlugin
  )

# ROS Wrapper for the Multirotor UAV System + controllers

add_library(MrsMultirotorSimulator_SimulatorSystemRos SHARED
  src/uav_system_ros.cpp
)

target_link_libraries(MrsMultirotorSimulator_SimulatorSystemRos
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsMultirotorSimulator_SimulatorSystemRos
  rclcpp
  rclcpp_components
  mrs_lib
  sensor_msgs
  std_msgs
  mrs_msgs
  nav_msgs
  ${Eigen_LIBRARIES}
)

# MRS Simulator

add_library(MrsMultirotorSimulator_Simulator SHARED
  src/multirotor_simulator.cpp
)

target_link_libraries(MrsMultirotorSimulator_Simulator
  MrsMultirotorSimulator_SimulatorSystemRos
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsMultirotorSimulator_Simulator
  rclcpp
  rclcpp_components
  mrs_lib
  sensor_msgs
  std_msgs
  mrs_msgs
  geometry_msgs
  nav_msgs
)

rclcpp_components_register_nodes(MrsMultirotorSimulator_Simulator PLUGIN "mrs_multirotor_simulator::MultirotorSimulator" EXECUTABLE MrsMultirotorSimulator_Simulator)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS
  MrsMultirotorSimulator_SimulatorSystemRos
  MrsMultirotorSimulator_Simulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# this will export the include-related variables for use by other libraries
# this is needed if you wish to include the headers from this pkg into downstream libraries
ament_export_include_directories("include")
ament_package()
