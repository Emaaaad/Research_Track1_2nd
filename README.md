# Research Track I - Second Assignment
### Student: Seyed Emad Razavi (5782734)
### Professor: Carmine Tommaso Recchiuto

## Overview
This project, the second assignment for the Research Track 1 course, involves the development of a ROS package to control a robot in a simulation environment. The package contains three nodes, each with specific functionalities for robot movement and data analysis.

## Nodes Description
- **node_a.py**: Implements an action client allowing the user to set or cancel a target position (x,y). It also publishes the robot's position and velocity based on `/odom` topic data.
- **node_b.py**: Provides a service to return the last target coordinates set by the user.
- **node_c.py**: Offers a service to calculate the distance of the robot from the target and the robot’s average speed, utilizing a custom message for the robot's position and velocity.

## Installation and Running
### Prerequisites
- ROS (Robot Operating System) installation, preferably ROS Noetic.
- The RT1_assignment_2 package.
- xterm.

### Installation Steps
1. Clone the RT1_assignment_2 package:

git clone https://github.com/LemmaMoto/RT1_assignment_2.git

markdown

2. Install xterm:

sudo apt-get -y install xterm

markdown

3. Ensure Python scripts are executable:

cd path/to/RT1_assignment_2/scripts
chmod +x node_a.py node_b.py node_c.py bug_as.py go_to_point_service.py wall_follow_service.py

markdown


### Running the Simulation
1. Launch the simulation:

roslaunch assignment_2_2023 assignment1.launch

shell


## Nodes Usage
### node_a.py
- Assign new targets or abort the existing ones for the robot.
- Broadcast the robot's current location and speed.

### node_b.py
- Provides the last desired position of the robot.
- To use, call the service:

rosservice call /input

vbnet


### node_c.py
- Provides average velocity and distance from the target.
- Subscribes to `/pos_vel` for updates.
- To use, call the service:

rosservice call /info_service

markdown


## Pseudocode
- [Include a brief pseudocode for each node, focusing on key functionalities.]

## Possible Improvements
- Implement a threshold for minor distance changes in node_c.
- Enhance the direction algorithm in node_a for optimal path calculation.

---

## Documentation
- Full documentation and additional details can be found [here](https://github.com/Vahidba72/ResearchTrack_assignment_2).

## Package Structure

### src Directory
- **go_to_point_service.py**: Node for moving the robot to a specified location.
- **wall_follow_service.py**: Node for wall-following behavior.
- **bug_as.py**: Node implementing an algorithm for goal reaching using point navigation and wall-following.
- **my_node_A.py**: Manages robot's position and velocity, dynamic goal setting, and publishes custom messages.
- **my_node_B.py**: Service node to retrieve last target coordinates.
- **my_node_C.py**: Service node for calculating average distance and velocity.

### msg Directory
- **Vel.msg**: Custom message for representing the robot's position and velocity.

### srv Directory
- **LastTarget.srv**: Service for retrieving the last target coordinates.
- **Average.srv**: Service for calculating average distance and velocity.

### launch Directory
- **assignment1.launch**: Launch file for starting the simulation with necessary configurations.

## Custom Message and Services
- **Vel.msg**:

Custom message structure for robot's position and velocity data.

    LastTarget.srv: Service definition for last target coordinates retrieval.
    Average.srv: Service for calculating and providing average distance and velocity data.

Usage

Follow these steps to run the package:

    Ensure ROS Noetic is installed.
    Clone the package into a ROS workspace.
    Build the package using catkin_make.
    Modify parameters in assignment1.launch as needed.
    Make Python scripts executable:

    bash

chmod +x name_of_the_file.py

Run the launch file:

    roslaunch assignment_2_2023 assignment1.launch

Nodes Functionality

    node_a.py: Interactive node for target setting and broadcasting position and velocity.
    node_b.py: Service node to provide last target coordinates.
    node_c.py: Service node for calculating distance to target and average velocity.

Additional Notes

    For real-time updates and additional resources, visit the GitHub repository.
    This project is part of the Research Track 1 course under the guidance of Professor Carmine Tommaso Recchiuto.

Feel free to contribute or suggest improvements!
