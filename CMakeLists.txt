cmake_minimum_required(VERSION 3.1.2)
project(mrs_multirotor_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_compile_options("-Og")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_msgs
  mrs_lib
  nodelet
  std_msgs
  )

set(LIBRARIES
  MrsMultirotorSimulator
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  INCLUDE_DIRS include include/controllers
  CATKIN_DEPENDS std_msgs mrs_msgs mrs_lib roscpp std_srvs
  )

include_directories(
  include
  include/controllers
  ${catkin_INCLUDE_DIRS}
  )

add_library(MrsMultirotorSimulator
  src/multirotor_simulator.cpp
  src/quadrotor_model.cpp

  src/controllers/rate_controller.cpp
  )

add_dependencies(MrsMultirotorSimulator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsMultirotorSimulator
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

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
