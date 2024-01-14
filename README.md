# Research Track I Second Assignment
## Student: [Seyed Emad Razavi] ([5782734]), Professor: Carmine Tommaso Recchiuto

### Introduction
This is the second assignment for the Research Track 1 course. The assignment involves creating a new package containing three nodes to control robot movement in a specific environment and gather relevant data. The package also includes a launch file to start the simulation.

### Nodes Description
1. **Node A**: An action client allowing the user to set or cancel a target (x,y). It publishes the robot's position and velocity (x,y,vel_x,vel_z) using data from the `/odom` topic.
2. **Node B**: A service node that returns the coordinates of the last target set by the user.
3. **Node C**: A service node that subscribes to the robot's position and velocity and provides the distance from the target and the robot's average speed.

### Installation and Running
#### Prerequisites
- ROS installation.
- RT1_assignment_2 package.
- xterm installation.

#### Setup
1. Clone the RT1_assignment_2 package: git clone https://github.com/Emaaaad/Research_Track1_2nd

2. Install xterm:
   sudo apt-get -y install xterm

