#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mrs_msgs.msg import HwApiControlGroupCmd

class FlightPlanner(Node):
    def __init__(self):
        super().__init__('flight_planner')
        
        self.odom_sub = self.create_subscription(Odometry, '/multirotor_simulator/uav1/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(HwApiControlGroupCmd, '/multirotor_simulator/uav1/control_group_cmd', 10)
        
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.current_z = 0.0
        self.error_sum = 0.0
        self.last_error = 0.0

    def odom_callback(self, msg):
        self.current_z = msg.pose.pose.position.z

    def control_loop(self):
        target_z = 60.0 
        dt = 0.02

        kp = 0.1
        ki = 0.02
        kd = 0.04

        error = target_z - self.current_z
        self.error_sum += error * dt
        d_error = (error - self.last_error) / dt
        self.last_error = error
        p_term = kp * error
        i_term = ki * self.error_sum
        d_term = kd * d_error
        control_output = p_term + i_term + d_term
        control_output = max(min(control_output, 0.35), -0.35)

        cmd = HwApiControlGroupCmd()
        cmd.stamp = self.get_clock().now().to_msg()
        
        cmd.throttle = 12.0  
        cmd.pitch = control_output 
        cmd.roll = 0.0 
        cmd.yaw = 0.0  
        
        self.get_logger().info(f"Altitude: {self.current_z:.2f} m | Pitch cmd: {cmd.pitch:.3f} rad")
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = FlightPlanner()
    print("Starting flight planner")
    
    # Blokujici smyčka
    rclpy.spin(node)

if __name__ == '__main__':
    main()