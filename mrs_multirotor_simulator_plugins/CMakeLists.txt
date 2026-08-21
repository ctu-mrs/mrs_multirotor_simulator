cmake_minimum_required(VERSION 3.15)
project(mrs_multirotor_simulator_plugins)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  mrs_lib
  pluginlib
  Eigen3
  mrs_multirotor_simulator_core
  mrs_multirotor_simulator
)

set(LIBRARIES
  MrsMultirotorSimulatorPlugins_RandomizePositionWorldPlugin
  MrsMultirotorSimulatorPlugins_BoidsUavPlugin
  MrsMultirotorSimulatorPlugins_NeighborCountUavPlugin
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${mrs_lib_INCLUDE_DIRS}
  ${mrs_multirotor_simulator_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

# Randomize Position World Plugin

add_library(MrsMultirotorSimulatorPlugins_RandomizePositionWorldPlugin SHARED
  src/randomize_position_world_plugin.cpp
  )

target_link_libraries(MrsMultirotorSimulatorPlugins_RandomizePositionWorldPlugin
  rclcpp::rclcpp
  rclcpp_components::component
  mrs_lib::mrs_lib
  pluginlib::pluginlib
  Eigen3::Eigen
  mrs_multirotor_simulator_core::mrs_multirotor_simulator_core
  mrs_multirotor_simulator::MrsMultirotorSimulator_SimulatorSystemRos
  )

# Boids UAV Plugin

add_library(MrsMultirotorSimulatorPlugins_BoidsUavPlugin SHARED
  src/boids_uav_plugin.cpp
  )

target_link_libraries(MrsMultirotorSimulatorPlugins_BoidsUavPlugin
  rclcpp::rclcpp
  rclcpp_components::component
  mrs_lib::mrs_lib
  pluginlib::pluginlib
  Eigen3::Eigen
  mrs_multirotor_simulator_core::mrs_multirotor_simulator_core
  mrs_multirotor_simulator::MrsMultirotorSimulator_SimulatorSystemRos
  )

# Neighbor Count UAV Plugin

add_library(MrsMultirotorSimulatorPlugins_NeighborCountUavPlugin SHARED
  src/neighbor_count_uav_plugin.cpp
  )

target_link_libraries(MrsMultirotorSimulatorPlugins_NeighborCountUavPlugin
  rclcpp::rclcpp
  rclcpp_components::component
  mrs_lib::mrs_lib
  pluginlib::pluginlib
  mrs_multirotor_simulator_core::mrs_multirotor_simulator_core
  mrs_multirotor_simulator::MrsMultirotorSimulator_SimulatorSystemRos
  )

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_multirotor_simulator plugins.xml)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
