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

## Prerequisites
- ROS (Robot Operating System) [Recommended Version: Noetic]
- Python 3.x
- Relevant ROS packages: `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf`, `actionlib`, `std_srvs`

## Installation
Clone the repository into your ROS workspace:
```bash
cd ~/catkin_ws/src
```
```bash
git clone https://github.com/Emaaaad/Research_Track1_2nd.git
```

## Permissions for running
Ensure executable permissions for Python files inside the 'scripts' folder:

```bash
    chmod +x scripts/nodeA.py
```
```bash
    chmod +x scripts/nodeB.py
```
```bash
    chmod +x scripts/nodeC.py
```
```bash
    chmod +x scripts/bug_as.py
```
```bash
    chmod +x scripts/go_to_point_service.py
```
```bash
    chmod +x scripts/wall_follow_service.py
```

### Setting Up and Executing the Project
The project requires a functioning ROS environment, access to this [GitHub Repository](https://github.com/Emaaaad/Research_Track1_2nd.git), and xterm installed on your system.

Execute the following commands in your terminal for setup and execution:

```bash
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

**Pseudocode for Node A:**
```bash
```python
Class GoalHandler:
    Initialize node, publishers, and action client
    Process goals:
        While ROS is running:
            Subscribe to Odometry data
            Get user input for goal management
            Set or cancel goals based on user input
            Publish velocity and position data

   Main:
      Initialize ROS node
      Create instance of GoalHandler
      Run the goal processing loop
```

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

