System Nodes
============

This section provides details on the system nodes developed for the ROS package, focusing on their roles within the robotic navigation system.

my_node_A Module (node_a.py)
-----------------------------

**synopsis**:

Python module for the assignment_2_2023.

**Description**:

A node that implements an action client, allowing the user to set a target (x, y) or to cancel an already given target. The node also publishes the robot's position and velocity as a custom message (x, y, vel_x, vel_z), by relying on the values published on the topic `/odom`.

Subscribes to:

- `/odom`

Publishes to:

- `/kinematics`

Service:

- None

Action client:

- `/reaching_goal`

**Functions**:

- **main():** Main function to make the robot move towards different targets

    Parameters:
    
        none
        
    Returns:
    
        none
    
    In the main function, the node_a is first initialized and the target x and y positions (x_t and y_t) are extracted from the launch file (assignment1.launch). Then a question is asked from the user to decide whether he wants to change the goal or cancel the current goal. If 'Cancel' is chosen, the robot will stop moving. If 'Change' is chosen, the robot asks for the new x and y targets and then sends the new targets (goal_new) to the launch file and to the client.

- **publish_kinematics(msg):** Callback function of the topic `/odom` to calculate the position and velocity of the robot

    Parameters:
    
        - msg (Pose): Robot’s position and velocity information
        
    Returns:
    
        none

    This function gets the required odometry information of the robot using the `/odom` topic and then extracts the x and y positions of the robot and also linear and angular velocities of the robot and publishes them on the topic `/kinematics` in the `pos_vel` variable.

**Usage**:

.. code-block:: bash

    rosrun assignment_2_2023 node_a.py


my_node_B Module (node_b.py)
-----------------------------

**synopsis**:

A service node for the assignment_2_2023 to show the last target.

**Description**:

This node gets the last imported target by the user from the launch file and displays it as the output when the service is called.

**Service**:

- `last_target`

**Functions**:


- **main():** Main function to initialize the node and the service.
- **put_target(req):** Callback function of the service `last_target` to extract the last chosen target.

    Parameters:
    
        - req (LastTarget): Request for the last target display
        
    Returns:
    
        - The x and y for the last chosen target
        
    Return type:
    
        - LastTargetResponse

    This function extracts the desired x and y positions (`des_pos_x` and `des_pos_y`) from `assignment1.launch` and stores them into the response variable of the service (`res`).

**Usage**:

.. code-block:: bash

    rosrun assignment_2_2023 node_b.py

    

my_node_C Module (node_c.py)
-----------------------------

**synopsis**:

A service node for the assignment_2_2023 to show the distance and average speed.

**Description**:

This node is a part of Assignment 2 for the year 2023. It calculates the distance and average speed based on received messages from the `/kinematics` topic. The calculations involve calling the `last_target` service to retrieve the target coordinates and then processing the incoming velocity messages to compute the distance and average speed.

**Service**:

- `average`
- `last_target`

**Functions**:

- **main():** Main function for the service average
- **give_avg(req):** Callback function of the service `average` to compute the average distance and velocity.

    Parameters:
    
        - req (AverageRequest): Average request.
        
    Returns:
    
        - Distance to target and average velocity.
        
    Return type:
    
        - AverageResponse

- **get_average(msg):** Callback function of the topic `/kinematics` to compute the distance and average velocity.

    Parameters:
    
        - msg (Pose): Robot’s position and velocity information
        
    Returns:
    
        - None

    This function gets the position and velocity of the robot from the `/kinematics` topic and then based on the `average_window` parameter extracted from the launch file, computes the distance from the last chosen target and also computes the average linear velocity of the robot over the given window.

**Usage**:

.. code-block:: bash

    rosrun assignment_2_2023 node_c.py
