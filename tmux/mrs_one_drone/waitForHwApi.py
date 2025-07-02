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

    def callback(self, msg):
        self.got = True

    def ready(self):
        return self.got

    def getTopic(self):
        return self.topic

class MyNode(Node):

    def __init__(self):

        UAV_NAME = os.getenv('UAV_NAME')

        while UAV_NAME == None:
            print('[ERROR]: Environment variable UAV_NAME not set!')
            time.sleep(1.0)

        try:
            random_str = str(''.join(random.choices(string.ascii_uppercase + string.digits, k=8)))

            super().__init__("waitForHwApi_{}_{}".format(UAV_NAME, random_str))
        except:
            return

        topics = [
            "/" + UAV_NAME + "/hw_api/connected std_msgs/msg/Empty",
        ]

        self.checkers = []

        for topic in topics:
            self.checkers.append(TopicChecker(topic, self))

        self.timer = self.create_timer(1.0, self.doAction)

    def doAction(self):

        ready = True

        for checker in self.checkers:
            if not checker.ready():
                ready = False
                print("waiting for {}".format(checker.getTopic()))
                break

        if ready:
            rclpy.shutdown()

def main(args=None):

    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node)

if __name__ == '__main__':

    main()
