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

    def callback(self, data):
        rospy.loginfo("Got %s", data.data)

    def run(self):
        move_pub = rospy.Publisher('robot_move_info', String, queue_size=10)
        env_mod_pub = rospy.Publisher('env_mod', String, queue_size=10)
        rospy.Subscriber('assignments', String, self.callback)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            move_msg = "Robot agent moved to 42"
            env_msg = "Robot modified tile 42"
            move_pub.publish(move_msg)
            env_mod_pub.publish(env_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        obj = RobotAgent()
        obj.run()
    except rospy.ROSInterruptException:
        pass