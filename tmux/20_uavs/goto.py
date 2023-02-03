#!/usr/bin/python3

import rospy
import rosnode

from mrs_msgs.msg import HwApiPositionCmd as HwApiPositionCmd

class Goto:

    def __init__(self):

        rospy.init_node('goto', anonymous=True)

        publishers = []
        n_uavs = 100

        for i in range(0, n_uavs):
            publishers.append(rospy.Publisher('/multirotor_simulator/uav{}/position_cmd'.format(i+1), HwApiPositionCmd, queue_size=1))

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            msg = HwApiPositionCmd();
            msg.position.x = 0;
            msg.position.y = 0;
            msg.position.z = 5;
            msg.heading = 0;

            for i in range(0, n_uavs):
                publishers[i].publish(msg)

            rate.sleep();

if __name__ == '__main__':
    try:
        goto = Goto()
    except rospy.ROSInterruptException:
        pass
