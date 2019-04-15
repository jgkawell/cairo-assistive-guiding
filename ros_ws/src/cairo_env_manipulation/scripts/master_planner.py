import rospy
import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray

class MasterPlanner():

    def __init__(self):
        rospy.init_node('master', anonymous=False)
        rospy.logwarn("MASTER PLANNER: Not Implemented!")

    def run(self):
        assigner_pub = rospy.Publisher('assignments', String, queue_size=10)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            assign_msg = "Robot Agent 42 assigned to Human Agent 42"
            assigner_pub.publish(assign_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        obj = MasterPlanner()
        obj.run()
    except rospy.ROSInterruptException:
        pass
