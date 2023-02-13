#!/bin/bash

rostopic pub /multirotor_simulator/uav1/velocity_hdg_rate_cmd mrs_msgs/HwApiVelocityHdgRateCmd "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
velocity:
  x: 1.0
  y: 0.0
  z: 0.0
heading_rate: 1.0" -r 10
