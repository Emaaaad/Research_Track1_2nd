System Nodes
============

This section provides details on the system nodes developed for the ROS package, focusing on their roles within the robotic navigation system.

GoalHandler Node (nodeA.py)
---------------------------

**Description**: Manages target settings for robot navigation using an action client. It monitors feedback for target status and transmits the robot's positional and velocity data.

**Key Features**:

- Setting and cancellation of (x, y) coordinates
- Monitoring feedback from the action server
- Transmitting robot's positional and velocity data

**Usage**:

.. code-block:: bash

    rosrun your_package_name nodeA.py

Last Target Retrieval Node (nodeB.py)
-------------------------------------

**Description**: Provides a service to fetch the last set target coordinates for the robot, acting as a crucial component for target management within the navigation system.

**Key Features**:

- Fetching last set target coordinates
- Service-based communication

**Usage**:

.. code-block:: bash

    rosrun your_package_name nodeB.py

Data Processing Node (nodeC.py)
-------------------------------

**Description**: Subscribes to the robot's position and velocity data, calculating and returning the average speed and the distance from the set target through services.

**Key Features**:

- Subscribing to robot's position and velocity data
- Calculating average speed and distance from the target

**Usage**:

.. code-block:: bash

    rosrun your_package_name nodeC.py
