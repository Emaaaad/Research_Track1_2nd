Research Track I Second Assignment
=================================

Student: [Seyed Emad Razavi](https://github.com/Emaaaad) (5782734), Professor: [Carmine Tommaso Recchiuto](https://github.com/CarmineD8)
------------------------------------------------------------------------------------------------------------------------------------------

This is the second assignment for the Research Track 1 course. The assignment requires the creation of a new package containing three nodes to control robot movement in a specific environment and to gather relevant data. The nodes are as follows:

- A node that implements an action client, allowing the user to set a target (x,y) or to cancel it.
Try to use the feedback/status of the action server to know when the target has been reached.
The node also publishes the robot position and velocity as a custom message (x,y,vel_x,vel_z), by relying on the values published on the topic/odom;
- A service node that, when called, returns the coordinates of the last target sent by the user;
- Another service node that subscribes to the robot’s position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot’s average speed.

The assignment also necessitates the creation of a launch file to initiate the entire simulation.


Installing and running
----------------------

The simulator requires a ROS installation, this repository [RT1_assignment_2](https://github.com/Emaaaad/Research_Track1_2nd.git) and the xterm installation.

Execute the following commands in your terminal:

```bash
# Clone the RT1_assignment_2 package
$ git clone https://github.com/Emaaaad/Research_Track1_2nd.git

# Install xterm
$ sudo apt-get -y install xterm

# Make sure the Python files have execution permission
$ chmod +x node_a.py node_b.py node_c.py bug_as.py go_to_point_service.py wall_follow_service.py

# Run the program
$ roslaunch assignment_2_2023 assignment1.launch


Nodes
node_a.py

This Python script creates a ROS node for robot interaction within a ROS environment. It enables the user to assign new targets (by inputting 'y') or abort (by inputting 'c') the existing target for the robot, while also broadcasting the robot's current location and speed.
node_b.py

This Python script defines a ROS node that provides a service to return the last desired position of a robot. It provides a service named 'input' that returns the target positions when called.

To visualize the information, you can call the service using the following command in a new terminal:

bash

$ rosservice call /input

This will return the last desired x and y positions of the robot.
node_c.py

This Python script defines a ROS node that provides a service to return the average velocity and the distance between the current and desired positions of a robot. It subscribes to the /pos_vel topic to update these values and provides a service named 'info_service' that returns these values.
Pseudocode

    Import necessary libraries

    Define the GoalHandler class

        Initialize the class
            Create a publisher to the /pos_vel topic
            Create an action client for the /reaching_goal action server
            Wait for the action server to be available
            Initialize a flag to indicate if the current goal has been cancelled

        Define the handle_goal_commands method
            Loop until ROS is shutdown
                Subscribe to the /odom topic and call publish_position_velocity method when a message is received
                Prompt the user to enter a command
                Get the current target position from the parameter server
                Create a new goal with the current target position
                If the user command is 'y'
                    Prompt the user to enter the x and y coordinates for the new goal
                    If the input is valid, update the target position parameters and the goal
                    Send the new goal to the action server
                    Update the goal cancelled flag
                If the user command is 'c'
                    If there is an active goal, cancel it and update the goal cancelled flag
                Log the last received goal

        Define the publish_position_velocity method
            Extract the current position and velocity from the Odometry message
            Create a new Vel message with the current position and velocity
            Publish the Vel message

    Define the main function
        Initialize the node
        Create an instance of the GoalHandler class
        Call the handle_goal_commands method of the GoalHandler instance

    If the script is the main program, call the main function

Possible improvements

This section outlines potential enhancements for the code.

Currently, when node_c publishes a message before a goal is defined, the distance slightly increases over time. To address this, a threshold could be introduced to ignore minor changes.

At present, when the robot encounters a wall, it turns in a fixed direction, not necessarily towards the shortest path. An improvement could be to modify the algorithm so that the robot calculates the optimal direction to turn.
