# MRS Multirotor Simulator

The multirotor UAV dynamics simulator.

> :warning: **Attention please: This README needs work.**
>
> The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid.

This package provides a minimalistic dynamics simulator for multirotor UAVs.

## Features

* Header-only implementation of a single-UAV dynamics with feedback controllers. The header-only library can be using within your code for, e.g., reinforcement learning.
* Full rigid-body UAV dynamics stepped by an ODE solver.
* A cascade of feedback controllers that provide various references anywhere from individual actuator control up to desired 3D position + heading.
* A ROS wrapper for an individual UAV.
* A ROS simulator packed for multir-UAV simulation with mutual collisions and a ground plane.
* The barebone simulation work with hundreds of UAVs in real time.
