#!/usr/bin/python3

import time
import os
import random
import string

import rclpy
from rclpy.node import Node

from rosidl_runtime_py.utilities import get_message

class TopicChecker:

    def __init__(self, topic, node):

        self.topic = topic.split(' ')[0]

        msg_class = get_message(topic.split(' ')[1])

        self.sub = node.create_subscription(msg_class, self.topic, self.callback, 10)

        self.got = False

        self.msg = []

    def callback(self, msg):
        self.got = True
        self.msg = msg

    def ready(self):
        return self.got

    def getTopic(self):
        return self.topic

    def getMsg(self):
        return self.msg

class MyNode(Node):

    def __init__(self):

        UAV_NAME = os.getenv('UAV_NAME')

        while UAV_NAME == None:
            print('[ERROR]: Environment variable UAV_NAME not set!')
            time.sleep(1.0)

        try:
            random_str = str(''.join(random.choices(string.ascii_uppercase + string.digits, k=8)))

            super().__init__("waitForTakeoff_{}_{}".format(UAV_NAME, random_str))
        except:
            return

        self.topic = "/" + UAV_NAME + "/control_manager/diagnostics mrs_msgs/msg/ControlManagerDiagnostics"

        self.checker = TopicChecker(self.topic, self)

        self.timer = self.create_timer(1.0, self.doAction)

    def doAction(self):

        if self.checker.ready():
            if self.checker.getMsg().flying_normally:
                rclpy.shutdown()
            else:
                print('[Info]: waiting for takeoff to finish')
        else:
            print('[Info]: waiting for data on "{}"'.format(self.checker.getTopic()))

def main(args=None):

    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node)

if __name__ == '__main__':

    main()
