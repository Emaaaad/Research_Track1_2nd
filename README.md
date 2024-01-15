# Advanced Robotics Navigation Assignment
## Research Track I - Second Task
**Student:** [Seyed Emad Razavi](https://github.com/Emaaaad) (5782734)  
**Professor:** [Carmine Tommaso Recchiuto](https://github.com/CarmineD8)

---

### Project Description
This assignment in Research Track I course focuses on developing a ROS package to manage a robot's movement within a Gazebo simulation. The project encapsulates three ROS nodes, each contributing to different aspects of the robot's navigation and data processing.

### Node Overview
- **Navigation Node (Node A):** Manages target settings for robot navigation using an action client, allowing setting and cancellation of (x, y) coordinates. Monitors feedback from the action server for target status and also transmits robot's positional and velocity data.
- **Target Retrieval Node (Node B):** Provides a service to fetch the last set target coordinates for the robot.
- **Data Processing Node (Node C):** Subscribes to the robot's position and velocity data, offering services to calculate and return the average speed and the distance from the set target.


## Permissions for running
Ensure executable permissions for Python files inside the 'scripts' folder:

```bash
    chmod +x scripts/nodeA.py
    chmod +x scripts/nodeB.py
    chmod +x scripts/nodeC.py
    chmod +x scripts/bug_as.py
    chmod +x scripts/go_to_point_service.py
    chmod +x scripts/wall_follow_service.py
```

### Setting Up and Executing the Project
The project requires a functioning ROS environment, access to this [GitHub Repository](https://github.com/Emaaaad/Research_Track1_2nd.git), and xterm installed on your system.

Execute the following commands in your terminal for setup and execution:

```bash
# Clone the repository
$ git clone https://github.com/Emaaaad/Research_Track1_2nd.git

# Install xterm
$ sudo apt-get -y install xterm

# Change directory to the repository folder
$ cd Research_Track1_2nd

# Ensure scripts are executable
$ chmod +x src/assignment_2_2023/scripts/*.py

# Launch the simulation
$ roslaunch assignment_2_2023 start_simulation.launch
```

# System Nodes

## 1. GoalHandler Node - a_node.py

**Description**:
This Python script serves as a user interface client within the robotic system, enabling users to set goals and cancel ongoing tasks handled by an action server responsible for planning and executing robot movements. It relies on essential ROS modules and custom message types like Vel for position and velocity, as well as action messages like PlanningAction. The `GoalHandler` class initializes a ROS publisher (`/pos_vel` topic) for transmitting velocity and position data and an action client (`/reaching_goal` action server) for interacting with the goal planning system. It continuously listens to the `/odom` topic to acquire the robot's odometry data, allowing users to input commands ('s' for setting a new goal or 'q' for canceling the current goal). The script effectively processes these commands and updates position-velocity information on `/pos_vel`.

**Usage**:
Users can set new goals or cancel ongoing goals through this interface, utilizing the action client to communicate with the goal planning action server while simultaneously updating position-velocity data.


## 2. Last Target Service Node - b_node.py
**Description**:
This ROS node acts as a service client within the robotic system, catering to service requests related to the last desired x and y positions. The script imports requisite ROS modules and custom message types, particularly Vel and Input service messages. Inside the `LastTargetService` class, the script initializes class variables and provides an 'input' service using the Input service type. The `result_callback` function, serving as a callback for the service, retrieves the last desired x and y positions from ROS parameters and responds with this information. The node continues to run, ready to handle new service requests.

**Usage**:
Other components of the system can request the last desired x and y positions using the 'input' service, and this node responds with the requested information.

## 3. Info Service and Subscriber Node - c_node.py
**Description**:
This versatile ROS node performs the roles of both a service client and a subscriber within the robotic system. It is responsible for calculating the distance between desired and actual positions and computing the average velocity within a specified window. The script imports the necessary ROS modules, custom message types, and the math module for distance calculations. The `InfoService` class within the script initializes variables for tracking average velocity and distance, provides an 'info_service' service, and subscribes to the '/pos_vel' topic. The callback function, `get_distance_and_average_velocity`, extracts information regarding desired and actual positions, the velocity window size, and subsequently computes the distance and average velocity. Another callback function, part of the 'info_service' service, responds to requests by providing the calculated distance and average velocity. The node's `spin()` method ensures continuous operation, and the main function orchestrates the instantiation of the `InfoService` class.

**Usage**:
This node offers real-time information about distance and average velocity, serving as both a service provider for other system components and a subscriber to '/pos_vel' for continuous updates.


# Pseudocode for Robotic Goal Handler Node

# Import necessary libraries
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
from custom_messages.msg import Vel
from custom_messages.msg import PlanningAction, PlanningGoal, PlanningResult
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

# Define the GoalHandler class
class GoalHandler:
    def __init__(self):
        # Initialize class variables
        self.battery_level = 100  # Starting battery level as a percentage
        self.battery_pub = rospy.Publisher('/battery_level', Int32, queue_size=10)
        self.goal_action_client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.goal_action_client.wait_for_server()
        self.is_goal_active = False

    def handle_goal_commands(self):
        while not rospy.is_shutdown():
            # Subscribe to Odometry data and process it
            rospy.Subscriber("/odom", Odometry, self.odom_callback)
            
            # User interaction loop
            command = raw_input("Enter a command ('s' for set goal, 'q' for cancel goal): ")
            
            if command == 's':
                target_x = float(raw_input("Enter x coordinate for the new goal: "))
                target_y = float(raw_input("Enter y coordinate for the new goal: "))
                # Update target position parameters and create a new goal
                self.update_target_position(target_x, target_y)
                new_goal = self.create_goal(target_x, target_y)
                # Send the new goal to the action server
                self.send_goal(new_goal)
                self.is_goal_active = True
            elif command == 'q' and self.is_goal_active:
                # Cancel the current goal if one is active
                self.cancel_goal()
                self.is_goal_active = False
            else:
                print("Invalid command. Please try again.")
            
            # Log the last received goal
            self.log_last_goal()

    def odom_callback(self, odom_data):
        # Processing Odometry data
        # Extract position and velocity information
        position = odom_data.pose.pose.position
        linear_velocity = odom_data.twist.twist.linear
        angular_velocity = odom_data.twist.twist.angular
        # Create a Vel message and publish it
        self.publish_position_velocity(position, linear_velocity, angular_velocity)

    def update_target_position(self, x, y):
        # Update target position parameters on the parameter server
        rospy.set_param('/target_position/x', x)
        rospy.set_param('/target_position/y', y)

    def create_goal(self, x, y):
        # Create a new goal with the given x and y coordinates
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        return goal

    def send_goal(self, goal):
        # Send the goal to the action server for execution
        self.goal_action_client.send_goal(goal)

    def cancel_goal(self):
        # Cancel the current goal
        self.goal_action_client.cancel_goal()

    def log_last_goal(self):
        # Log the last received goal
        last_goal = self.goal_action_client.get_result()
        rospy.loginfo("Last Received Goal: {}".format(last_goal))

    def publish_position_velocity(self, position, linear_velocity, angular_velocity):
        # Publish position and velocity information
        position_and_velocity = Vel()
        position_and_velocity.pos_x = position.x
        position_and_velocity.pos_y = position.y
        position_and_velocity.vel_x = linear_velocity.x
        position_and_velocity.vel_z = angular_velocity.z
        self.battery_pub.publish(position_and_velocity)

# Define the main function
def main():
    rospy.init_node('goal_handler')
    goal_handler = GoalHandler()
    goal_handler.handle_goal_commands()

# Check if the script is the main program and call the main function
if __name__ == '__main__':
    main()

