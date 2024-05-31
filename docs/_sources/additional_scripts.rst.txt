Additional Scripts
==================

This section covers additional scripts that complement the functionality of the system nodes, enhancing the robot's navigation and data processing capabilities.

Bug Algorithm Script (bug_as.py)
--------------------------------

**Description**: Manages the robot's navigation and obstacle avoidance, subscribing to '/odom' for odometry data and '/scan' for laser scan data.

**Key Features**:

- Navigation and obstacle avoidance
- Subscribing to odometry and laser scan data

**Usage**:

.. code-block:: bash

    rosrun assignment_2_2023 bug_as.py

Go-to-Point Service (go_to_point_service.py)
--------------------------------------------

**Description**: Enables the robot to autonomously navigate to specified target locations, listening to odometry data to determine its current position and orientation.

**Key Features**:

- Autonomous navigation to target locations
- Listening to odometry data

**Usage**:

.. code-block:: bash

    rosrun assignment_2_2023 go_to_point_service.py

Wall Following Service (wall_follow_service.py)
------------------------------------------------

**Description**: Equips the robot with wall-following capabilities, processing laser scan data to navigate around obstacles.

**Key Features**:

- Wall-following navigation
- Processing laser scan data

**Usage**:

.. code-block:: bash

    rosrun assignment_2_2023 wall_follow_service.py
