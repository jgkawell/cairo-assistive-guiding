#!/usr/bin/env python
# license removed for brevity

import rospy
import numpy as np

from std_msgs.msg import String

class MasterPlanner():

    def __init__(self):
        rospy.init_node('planner', anonymous=False)
        rospy.logwarn("PLANNER: Not Implemented!")

        rospy.Subscriber('/world/update', String, self.update)
        assigner_pub = rospy.Publisher('/planner/assign', String, queue_size=10)

    def run(self):
        rospy.spin()

    def update(self, new_world):

        # TODO: Update local world knowledge

        return

    def path_predictor(self, world):
        
        # TODO: Predict the paths of all humans in the world

        # TODO: Determine which humans will violate the constraints

        return

    def abstractor(self, world):

        # TODO: Abstract world at current state

        return

    def candidate_generator(self, abstract_world, predictions):

        # TODO: Attempt to generate the desired state of the world

        # TODO: Return failure flag to abstractor if unsuccessful

        return

    def task_allocator(self, desired_world):

        # TODO: Attempt to assign a sequence of obstacles to each robot (within constraints)

        # TODO: Return failure flag to candidate_generator if unsuccessful

        return

if __name__ == '__main__':
    try:
        obj = MasterPlanner()
        obj.run()
    except rospy.ROSInterruptException:
        pass
