cmake_minimum_required(VERSION 3.28)
project(mrs_multirotor_simulator_core)

# The PROJECT_IS_TOP_LEVEL checks are used to support embedding into non-ROS
# projects.
# When embedded, we want to only provide the simulator target to link for the
# user. The embedded path should not use ament and should not build tests.
if(PROJECT_IS_TOP_LEVEL)
  message(STATUS
    "mrs_multirotor_simulator_core building ROS package."
  )

  # set the correct standards
  set(CMAKE_C_STANDARD 99)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  # set the compile options to show code warnings
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
  endif()
else()
  message(STATUS
    "mrs_multirotor_simulator_core NOT building ROS package."
  )
endif()

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(mrs_multirotor_simulator_core_mrs_multirotor_simulator_core STATIC) # EDITED
add_library(mrs_multirotor_simulator_core::mrs_multirotor_simulator_core ALIAS
  mrs_multirotor_simulator_core_mrs_multirotor_simulator_core
)
set_target_properties(mrs_multirotor_simulator_core_mrs_multirotor_simulator_core
  PROPERTIES
    EXPORT_NAME mrs_multirotor_simulator_core
    OUTPUT_NAME mrs_multirotor_simulator_core
    POSITION_INDEPENDENT_CODE ON # ADDED
)

target_sources(mrs_multirotor_simulator_core_mrs_multirotor_simulator_core
  PRIVATE
    "include/mrs_multirotor_simulator/uav_system/simulink_planner/FlightMechanicsModelForMRS.c"
  PUBLIC FILE_SET HEADERS BASE_DIRS "include"
  FILES
    # include
    # "include/KDTreeVectorOfVectorsAdaptor.h"
    # "include/nanoflann.hpp"
    # include/mrs_multirotor_simulator/uav_system
    # "include/mrs_multirotor_simulator/uav_system/multirotor_model.hpp"
    "include/mrs_multirotor_simulator/uav_system/uav_system.hpp"
    "include/mrs_multirotor_simulator/uav_system/fixedwing_model.hpp"
    "include/mrs_multirotor_simulator/uav_system/simulink_planner/FlightMechanicsModelForMRS.h"
    "include/mrs_multirotor_simulator/uav_system/simulink_planner/FlightMechanicsModelForMRS.h"

    #simulink_planner
    "include/mrs_multirotor_simulator/uav_system/simulink_planner/FlightMechanicsModelForMRS_private.h"
    "include/mrs_multirotor_simulator/uav_system/simulink_planner/FlightMechanicsModelForMRS_types.h"
    "include/mrs_multirotor_simulator/uav_system/simulink_planner/rtwtypes.h"
    
)

target_link_libraries(mrs_multirotor_simulator_core_mrs_multirotor_simulator_core
  PUBLIC
    Boost::headers
    Eigen3::Eigen
)

## --------------------------------------------------------------
## |                            test                            |
## --------------------------------------------------------------

if(PROJECT_IS_TOP_LEVEL AND ENABLE_TESTS)

  set_source_files_properties(
    "include/KDTreeVectorOfVectorsAdaptor.h"
    "include/nanoflann.hpp"
    PROPERTIES
      SKIP_LINTING ON
  )
  find_package(mrs_cmake REQUIRED)
  mrs_verify_interface_header_sets(
    mrs_multirotor_simulator_core_mrs_multirotor_simulator_core
  )

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(
    CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format"
    # List of paths to check
    # We need to specify them explicitly to exclude the nanoflann files.
    "${PROJECT_SOURCE_DIR}/include/mrs_multirotor_simulator"
  )
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

if(PROJECT_IS_TOP_LEVEL)

  find_package(ament_cmake REQUIRED)

  install(
    TARGETS
      mrs_multirotor_simulator_core_mrs_multirotor_simulator_core
    EXPORT "export_${PROJECT_NAME}"
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    FILE_SET HEADERS DESTINATION "include/${PROJECT_NAME}"
  )

  ament_export_targets("export_${PROJECT_NAME}" HAS_LIBRARY_TARGET)

  ament_export_dependencies(
    Boost
    Eigen3
  )

  ament_package()

endif()
