#!/usr/bin/env python3

import rospy
import actionlib
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult, PlanningFeedback

class PlanningActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('/reaching_goal', PlanningAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        rospy.loginfo("Goal received: x=%f, y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
        success = True
        feedback = PlanningFeedback()
        result = PlanningResult()

        # Simulate action execution (replace with your logic)
        for i in range(10):
            feedback.progress = i * 10
            self.server.publish_feedback(feedback)
            rospy.sleep(1)

        if success:
            result.result = True
            self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('planning_action_server')
    server = PlanningActionServer()
    rospy.spin()
