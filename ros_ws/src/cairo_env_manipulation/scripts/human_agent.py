#!/usr/bin/env python
# license removed for brevity

import rospy
import numpy as np

from std_msgs.msg import String

class HumanAgent():

    def __init__(self):
        rospy.init_node('human', anonymous=True)
        rospy.logwarn("HUMAN AGENT: Not implemented!")

        # TODO: generate agent_id (get from world?)
        self.agent_id = 1

        rospy.Subscriber('/world/turn', String, self.mover)
        move_pub = rospy.Publisher('/agent/move', String, queue_size=10)

        # TODO: Request world from world node (service: get_world)
        
    def run(self):
        rospy.spin()

    def planner(self):

        # TODO: Generate policy from world knowledge

        return

    def mover(self, agent_id):

        # TODO: Move if agent_id is your own

        # TODO: Move based on policy

        return


if __name__ == '__main__':
    try:
        obj = HumanAgent()
        obj.run()
    except rospy.ROSInterruptException:
        pass