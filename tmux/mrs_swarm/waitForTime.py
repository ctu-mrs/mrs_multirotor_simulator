#!/usr/bin/python3

import rospy
import rosnode
import time

class Node:

    def __init__(self):

        try:
            rospy.init_node('waitForTime', anonymous=True)
        except:
            return
            pass

        while not rospy.has_param('/use_sim_time') and not rospy.is_shutdown():

            rospy.loginfo('waiting for /use_sim_time')

            time.sleep(1.0)

        time.sleep(1.0)

if __name__ == '__main__':
    try:
        node = Node()
    except rospy.ROSInterruptException:
        pass
