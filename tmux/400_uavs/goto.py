#!/usr/bin/python3

import rospy
import rosnode
import random
import os

from mrs_msgs.msg import HwApiPositionCmd as HwApiPositionCmd

class Goto:

    def __init__(self):

        rospy.init_node('goto', anonymous=True)

        publishers = []
        n_uavs = 100

        for i in range(0, n_uavs):
            publishers.append(rospy.Publisher('/multirotor_simulator/uav{}/position_cmd'.format(i+1), HwApiPositionCmd, queue_size=1))

        rospy.sleep(2.0)

        rate = rospy.Rate(6)

        xs = []
        ys = []

        for i in range(0, n_uavs):
            xs.append(random.randint(20, 20))
            ys.append(random.randint(20, 20))
            # xs.append(0)
            # ys.append(0)

        while not rospy.is_shutdown():

            msg = HwApiPositionCmd();
            msg.position.z = 5;
            msg.heading = 0;

            for i in range(0, n_uavs):

                msg.position.x = xs[i]
                msg.position.y = ys[i]

                publishers[i].publish(msg)

            rate.sleep();

if __name__ == '__main__':
    try:
        goto = Goto()
    except rospy.ROSInterruptException:
        pass
