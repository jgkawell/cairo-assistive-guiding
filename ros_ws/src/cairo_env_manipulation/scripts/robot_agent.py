#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray

class RobotAgent():

    def __init__(self):
        rospy.init_node('robot', anonymous=True)
        rospy.logwarn("ROBOT AGENT: Not implemented!") 
        
    def run(self):
        while(not rospy.is_shutdown()):
            # keep running to check for info on sub
            x = 0

if __name__ == '__main__':
    try:
        obj = RobotAgent()
        obj.run()
    except rospy.ROSInterruptException:
        pass