#!/usr/bin/python3

import rospy
import rosnode

from mrs_msgs.msg import HwApiVelocityCmd as HwApiVelocityCmd

class Up:

    def __init__(self):

        rospy.init_node('Up', anonymous=True)

        publishers = []
        n_uavs = 100

        for i in range(0, n_uavs):
            publishers.append(rospy.Publisher('/multirotor_simulator/uav{}/velocity_cmd'.format(i+1), HwApiVelocityCmd, queue_size=1))

        rospy.sleep(2.0)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            msg = HwApiVelocityCmd();
            msg.velocity.x = 0;
            msg.velocity.y = 0;
            msg.velocity.z = 0;
            msg.heading = 0;

            for i in range(0, n_uavs):
                publishers[i].publish(msg)

            rate.sleep();

if __name__ == '__main__':
    try:
        Up = Up()
    except rospy.ROSInterruptException:
        pass
