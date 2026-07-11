# MRS Multirotor Simulator

The multirotor UAV dynamics simulator.

> :warning: **Attention please: This README needs work.**
>
> The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid.

This package provides a minimalistic dynamics simulator for multirotor UAVs.

The project is split into two ROS packages:
- `mrs_multirotor_simulator_core`
  - provides the core of the simulator
  - ROS independent, but built as ROS package by default for convenient including in ROS projects
  - can be embedded into non-ros projects
    - include e.g. as submodule
    - update your CMake with:
      ```cmake
      add_subdirectory(<path/to/repo/root>/mrs_multirotor_simulator_core)
      ...
      target_link_libraries(my_target PRIVATE
        mrs_multirotor_simulator_core::mrs_multirotor_simulator_core
      )
      ```
      or just include the headers manually
- `mrs_multirotor_simulator`
  - contains ROS wrappers for the simulator

## Features

* Header-only implementation of a single-UAV dynamics with feedback controllers. The header-only library can be using within your code for, e.g., reinforcement learning.
* Full rigid-body UAV dynamics stepped by an ODE solver.
* A cascade of feedback controllers that provide various references anywhere from individual actuator control up to desired 3D position + heading.
* A ROS wrapper for an individual UAV.
* A ROS simulator packed for multir-UAV simulation with mutual collisions and a ground plane.
* The barebone simulation work with hundreds of UAVs in real time.
