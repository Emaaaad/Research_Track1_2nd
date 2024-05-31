#!/usr/bin/env python3
import rospy
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Input, InputResponse

# Service Class for Handling Last Target Data
class LastTargetHandler:
    def __init__(self):
        # Storing Last Target Coordinates
        self.prev_target_x = 0
        self.prev_target_y = 0

        # Node Initialization
        rospy.init_node('target_position_service')
        rospy.loginfo("Service for retrieving last target position initialized")

        # Setting Up ROS Service
        self.service = rospy.Service('retrieve_last_target', Input, self.handle_last_target)

    # Service Callback for Providing Last Target Data
    def handle_last_target(self, _):
        # Response Message Construction
        response = InputResponse()
        # Retrieving and Assigning Last Target Coordinates
        self.prev_target_x = rospy.get_param('/des_pos_x')
        self.prev_target_y = rospy.get_param('/des_pos_y')
        response.input_x = self.prev_target_x
        response.input_y = self.prev_target_y

        # Sending Back Response
        return response

    # Node's Main Loop
    def run_service(self):
        rospy.spin()

# Main Function Entry Point
if __name__ == "__main__":
    # Initializing Service Class Instance
    target_service = LastTargetHandler()
    # Running the Service
    target_service.run_service()