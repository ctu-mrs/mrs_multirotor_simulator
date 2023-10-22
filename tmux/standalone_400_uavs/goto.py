#!/usr/bin/python3

import rospy
import rosnode
import random
import os

from mrs_msgs.msg import HwApiPositionCmd as HwApiPositionCmd

class Goto:

    def __init__(self):

        rospy.init_node('goto', anonymous=True)

        rospy.loginfo('ros not initialized')

        publishers = []
        n_uavs = 400

        rospy.loginfo('setting up publishers')

        for i in range(0, n_uavs):
            publishers.append(rospy.Publisher('/multirotor_simulator/uav{}/position_cmd'.format(i+1), HwApiPositionCmd, queue_size=1))

        xs = []
        ys = []
        zs = []
        hdgs = []

        for i in range(0, n_uavs):

            # random position
            xs.append(random.uniform(-40, 40))
            ys.append(random.uniform(-40, 40))
            zs.append(random.uniform(2, 20))
            hdgs.append(random.uniform(-3.14, 3.14))

            # # particular position
            # xs.append(0)
            # ys.append(0)
            # zs.append(5)
            # hdgs.append(0)

        rospy.loginfo('publishing')

        msg = HwApiPositionCmd();

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            for i in range(0, n_uavs):

                msg.position.x = xs[i]
                msg.position.y = ys[i]
                msg.position.z = zs[i]
                msg.heading = hdgs[i]

                publishers[i].publish(msg)

            rate.sleep();

if __name__ == '__main__':
    try:
        goto = Goto()
    except rospy.ROSInterruptException:
        pass
