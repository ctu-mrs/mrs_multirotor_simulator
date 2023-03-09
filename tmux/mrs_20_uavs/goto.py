#!/usr/bin/python3

import rospy
import rosnode
import random
import os

from mrs_msgs.msg import ReferenceStamped as ReferenceStamped

class Goto:

    def __init__(self):

        rospy.init_node('goto', anonymous=True)

        publishers = []
        n_uavs = 20

        for i in range(0, n_uavs):
            publishers.append(rospy.Publisher('/uav{}/control_manager/reference'.format(i+1), ReferenceStamped, queue_size=1))

        rospy.sleep(2.0)

        rate = rospy.Rate(6)

        xs = []
        ys = []

        for i in range(0, n_uavs):
            xs.append(random.randint(-40, 40))
            ys.append(random.randint(-40, 40))
            # xs.append(0)
            # ys.append(0)

        while not rospy.is_shutdown():

            msg = ReferenceStamped();
            msg.reference.position.z = 3;
            msg.reference.heading = 0;

            for i in range(0, n_uavs):

                msg.reference.position.x = xs[i]
                msg.reference.position.y = ys[i]

                publishers[i].publish(msg)

            rate.sleep();

if __name__ == '__main__':
    try:
        goto = Goto()
    except rospy.ROSInterruptException:
        pass
