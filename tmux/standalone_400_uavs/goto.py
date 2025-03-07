#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from mrs_msgs.msg import HwApiPositionCmd
import random

class Goto(Node):

    def __init__(self):

        super().__init__('goto')

        self.get_logger().info('ROS2 node initialized')

        self.pub = []
        self.n_uavs = 400

        self.get_logger().info('Setting up publishers')

        for i in range(self.n_uavs):
            topic_name = f'/multirotor_simulator/uav{i+1}/position_cmd'
            self.pub.append(self.create_publisher(HwApiPositionCmd, topic_name, 10))

        # self.xs = [random.uniform(-40, 40) for _ in range(self.n_uavs)]
        # self.ys = [random.uniform(-40, 40) for _ in range(self.n_uavs)]
        # self.zs = [random.uniform(2, 20) for _ in range(self.n_uavs)]
        # self.hdgs = [random.uniform(-3.14, 3.14) for _ in range(self.n_uavs)]

        self.xs = [0.0 for _ in range(self.n_uavs)]
        self.ys = [0.0 for _ in range(self.n_uavs)]
        self.zs = [10.0 for _ in range(self.n_uavs)]
        self.hdgs = [0.0 for _ in range(self.n_uavs)]

        self.timer = self.create_timer(0.05, self.publish_positions)  # 10Hz

        self.get_logger().info('Publishing started')

    def publish_positions(self):

        self.get_logger().info('Publishing')

        msg = HwApiPositionCmd()

        for i in range(self.n_uavs):
            msg.position.x = self.xs[i]
            msg.position.y = self.ys[i]
            msg.position.z = self.zs[i]
            msg.heading = self.hdgs[i]

            self.pub[i].publish(msg)

def main(args=None):

    rclpy.init(args=args)

    node = Goto()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
