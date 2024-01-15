# Advanced Robotics Navigation Assignment
## Research Track I - Second Task
**Student:** [Seyed Emad Razavi](https://github.com/Emaaaad) (5782734)  
**Mentor:** [Carmine Tommaso Recchiuto](https://github.com/CarmineD8)

---

### Project Description
This assignment in Research Track I course focuses on developing a ROS package to manage a robot's movement within a Gazebo simulation. The project encapsulates three ROS nodes, each contributing to different aspects of the robot's navigation and data processing.

### Node Overview
- **Navigation Node (Node A):** Manages target settings for robot navigation using an action client, allowing setting and cancellation of (x, y) coordinates. Monitors feedback from the action server for target status and also transmits robot's positional and velocity data.
- **Target Retrieval Node (Node B):** Provides a service to fetch the last set target coordinates for the robot.
- **Data Processing Node (Node C):** Subscribes to the robot's position and velocity data, offering services to calculate and return the average speed and the distance from the set target.

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
