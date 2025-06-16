#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from mrs_msgs.srv import ReferenceStampedSrv, Vec4
import os

class Goto(Node):

    def __init__(self):

        super().__init__('goto')

        self.get_logger().info('ROS2 node initialized')

        self.get_logger().info('Setting up the client')

        uav_name = os.environ['UAV_NAME']
        uav_name = "uav1"

        self.client = self.create_client(ReferenceStampedSrv, "/{}/control_manager/reference".format(uav_name))

        while not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('service not available, waiting again...')

        request = ReferenceStampedSrv.Request()
        request.header.frame_id = "world_origin"
        request.reference.position.x = 0.0
        request.reference.position.y = 0.0
        request.reference.position.z = 2.0
        request.reference.heading = 0.0

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

        self.get_logger().info('Service called')

        rclpy.shutdown()

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
