#!/usr/bin/env python3

import rospy
import math
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import vel_avg, vel_avgResponse

# Position and Velocity Analysis Service Class
class PosVelAnalyser:
    def __init__(self):
        # Variables for Computation
        self.avg_velocity_x = 0
        self.total_distance = 0

        # ROS Node Initialization
        rospy.init_node('position_velocity_analyser')
        rospy.loginfo("Position and Velocity Analysis Service Node Started")

        # ROS Service and Subscriber Setup
        self.service = rospy.Service("position_velocity_analysis", vel_avg, self.calculate_statistics)
        rospy.Subscriber("/pos_vel", Vel, self.update_stats)

    # Subscriber Callback for Position and Velocity Data
    def update_stats(self, data):
        # Fetching Target Position and Velocity Window Size
        target_x = rospy.get_param('/des_pos_x')
        target_y = rospy.get_param('/des_pos_y')
        velocity_window = rospy.get_param('/window_size')
        
        # Calculating Distance to Target Position
        self.total_distance = math.hypot(data.pos_x - target_x, data.pos_y - target_y)

        # Calculating Average Velocity (X Component)
        velocity_samples = [data.vel_x] if not isinstance(data.vel_x, list) else data.vel_x[-velocity_window:]
        self.avg_velocity_x = sum(velocity_samples) / min(len(velocity_samples), velocity_window)

    # Service Callback for Providing Computed Statistics
    def calculate_statistics(self, _):
        # Constructing Response with Calculated Data
        return vel_avgResponse(self.total_distance, self.avg_velocity_x)

    # Run the ROS Node
    def run_node(self):
        rospy.spin()

# Main Function
if __name__ == "__main__":
    # Service Initialization
    analyser_service = PosVelAnalyser()
    distance_velocity_service = rospy.ServiceProxy('position_velocity_analysis', vel_avg)

    while not rospy.is_shutdown():
        # Service Call for Statistics
        stats_response = distance_velocity_service()
        rospy.loginfo(f"Analysis Service Response:\n {stats_response}")

    # Running the Service Node
    analyser_service.run_node()