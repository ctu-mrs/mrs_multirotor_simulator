#!/usr/bin/python3

import rospy
import rosnode
import random

from mrs_msgs.msg import HwApiVelocityHdgRateCmd as HwApiVelocityHdgRateCmd

class Up:

    def __init__(self):

        rospy.init_node('velocity_cmd', anonymous=True)

        rospy.loginfo('ros node initialized')

        publishers = []
        n_uavs = 400

        rospy.loginfo('setting up publishers')

        for i in range(0, n_uavs):
            publishers.append(rospy.Publisher('/multirotor_simulator/uav{}/velocity_hdg_rate_cmd'.format(i+1), HwApiVelocityHdgRateCmd, queue_size=1))

        rate = rospy.Rate(10)

        xvel = []
        yvel = []
        zvel = []
        hdgvel = []

        for i in range(0, n_uavs):

            # random position
            xvel.append(random.uniform(-2, 2))
            yvel.append(random.uniform(-2, 2))
            zvel.append(random.uniform(0, 2))
            hdgvel.append(random.uniform(-1, 1))

            # # particular position
            # xvel.append(1.0)
            # yvel.append(0.0)
            # zvel.append(0.0)
            # hdgvel.append(0.0)

        rospy.loginfo('publishing')

        while not rospy.is_shutdown():

            for i in range(0, n_uavs):

                msg = HwApiVelocityHdgRateCmd();
                msg.velocity.x = xvel[i];
                msg.velocity.y = yvel[i];
                msg.velocity.z = zvel[i];
                msg.heading_rate = hdgvel[i];

                publishers[i].publish(msg)

            rate.sleep();

if __name__ == '__main__':
    try:
        Up = Up()
    except rospy.ROSInterruptException:
        pass
