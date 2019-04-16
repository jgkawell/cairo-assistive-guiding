#!/usr/bin/env python
# license removed for brevity

import rospy
import numpy as np

from std_msgs.msg import String

class RobotAgent():

    def __init__(self, abstract=True, probabilistic=False):
        rospy.init_node('robot', anonymous=True)
        rospy.logwarn("ROBOT AGENT: Not implemented!")

        # TODO: generate agent_id (get from world?)
        self.agent_id = 0

        move_pub = rospy.Publisher('/agent/move', String, queue_size=10)
        modify_pub = rospy.Publisher('/agent/modify', String, queue_size=10)

        rospy.Subscriber('/world/turn', String, self.mover)
        rospy.Subscriber('/planner/assign', String, self.planner)

        # TODO: Request world from world node (service: get_world)

    def run(self):
        rospy.spin()

    def planner(self, assignment):

        # TODO: Generate policy to accomplish assignment

        return

    def mover(self, agent_id):

        # TODO: Move if agent_id is your own

        # TODO: Decide next move from policy

        return

if __name__ == '__main__':
    try:
        obj = RobotAgent()
        obj.run()
    except rospy.ROSInterruptException:
        pass