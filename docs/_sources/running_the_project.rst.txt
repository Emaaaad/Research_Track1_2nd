Running the Project
===================

1. Build the Project
--------------------

.. code-block:: bash

    cd ~/catkin_ws
    catkin_make

2. Start the ROS Master
-----------------------

.. code-block:: bash

    roscore

3. Launch the Project
---------------------

.. code-block:: bash

    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch assignment_2_2023 start_simulation.launch
