#!/bin/bash

rostopic pub /multirotor_simulator/uav1/position_cmd mrs_msgs/HwApiPositionCmd "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
position:
  x: 0.0
  y: 0.0
  z: 3.0
heading: 0.0" -r 10
