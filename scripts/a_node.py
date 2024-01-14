#!/usr/bin/env python3

# ROS and Message Types
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

# GoalHandler Class
class GoalHandler:
    def __init__(self):
        # ROS Publishers and Clients
        self.vel_publisher = rospy.Publisher("/pos_vel", Vel, queue_size=1)
        self.goal_action_client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
        self.goal_action_client.wait_for_server()
        self.is_goal_active = False  # Track active goals

    def process_goals(self):
        while not rospy.is_shutdown():
            # Subscribe to Odometry data and process it
            rospy.Subscriber("/odom", Odometry, self.odom_callback)
            # User interaction for goal management
            user_input = input("Enter 'y' to set goal, 'c' to cancel goal: ")
            # Fetching target coordinates from parameters
            target_x = rospy.get_param('/des_pos_x')
            target_y = rospy.get_param('/des_pos_y')

            # Assembling a new goal
            new_goal = assignment_2_2023.msg.PlanningGoal()
            new_goal.target_pose.pose.position.x = target_x
            new_goal.target_pose.pose.position.y = target_y
            rospy.loginfo("Processing goal: X = %f, Y = %f", target_x, target_y)

            if user_input == 'y':
                # User input for new goal coordinates
                try:
                    new_x = float(input("New X-coordinate: "))
                    new_y = float(input("New Y-coordinate: "))
                except ValueError:
                    rospy.logwarn("Invalid input. Enter numeric values.")
                    continue

                # Update ROS parameters with new coordinates
                rospy.set_param('/des_pos_x', new_x)
                rospy.set_param('/des_pos_y', new_y)
                new_goal.target_pose.pose.position.x = new_x
                new_goal.target_pose.pose.position.y = new_y
                
                # Dispatching the goal to action server
                self.goal_action_client.send_goal(new_goal)
                self.is_goal_active = True

            elif user_input == 'c':
                # Cancel the ongoing goal
                if self.is_goal_active:
                    self.is_goal_active = False
                    self.goal_action_client.cancel_goal()
                    rospy.loginfo("Goal cancelled")
                else:
                    rospy.loginfo("No active goal to cancel")
            else:
                rospy.logwarn("Please enter 'y' to set a goal or 'c' to cancel.")

            rospy.loginfo("Goal status: X = %f, Y = %f", new_goal.target_pose.pose.position.x, new_goal.target_pose.pose.position.y)

    def odom_callback(self, odom_data):
        # Processing Odometry data
        position = odom_data.pose.pose.position
        linear_velocity = odom_data.twist.twist.linear
        angular_velocity = odom_data.twist.twist.angular

        # Preparing Vel message
        position_and_velocity = Vel()
        position_and_velocity.pos_x = position.x
        position_and_velocity.pos_y = position.y
        position_and_velocity.vel_x = linear_velocity.x
        position_and_velocity.vel_z = angular_velocity.z

        # Publishing Velocity and Position
        self.vel_publisher.publish(position_and_velocity)

def main():
    # Node Initialization
    rospy.init_node('target_goal_manager')
    goal_manager = GoalHandler()
    goal_manager.process_goals()

if __name__ == '__main__':
    main()
