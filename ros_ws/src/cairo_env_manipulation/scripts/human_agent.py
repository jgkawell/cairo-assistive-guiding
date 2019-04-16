#!/usr/bin/env python
# license removed for brevity

import rospy
import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray

class HumanAgent():

    def __init__(self):
        rospy.init_node('human', anonymous=True)
        rospy.logwarn("HUMAN AGENT: Not implemented!") 
        
    def run(self):
        move_pub = rospy.Publisher('/agent/move', String, queue_size=10)
        rate = rospy.Rate(10)
        while(not rospy.is_shutdown()):
            # keep running to check for info on sub
            x = 0
            move_msg = "Human moved to tile 42"
            move_pub.publish(move_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        obj = HumanAgent()
        obj.run()
    except rospy.ROSInterruptException:
        pass