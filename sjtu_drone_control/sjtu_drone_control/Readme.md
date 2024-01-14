1. **drone_utils/drone_object.py**: This file defines the `DroneObject` class. The class includes methods for controlling the drone, including taking off, landing, and moving in various directions.

2. **open_loop_control.py**: This script implements an open-loop control system for the drone. Open-loop control refers to sending commands to the drone without any feedback mechanism to adjust the commands based on the drone's response. The file provides functions to control the drone's movement, such as moving forward, turning, ascending, or descending. The drone's movement is controlled by adjusting the drone's roll, pitch, yaw, and vertical velocity. To run the script, use the following command:

   ```bash
   ros2 run sjtu_drone_control open_loop_control.py --task <'square', 'triangle', 'forward', 'backward', 'left', 'right', 'up', 'down'> --distance <float [m]> --number <int>
   ```


3. **drone_position_control.py**: This file contains the implementation of a position control system for the drone. Unlike open-loop control, position control utilizes the gazebo plugin and PID controllers to move the drone to the desired position. To run the script, use the following command:

   ```bash
   ros2 run sjtu_drone_control drone_position_control.py
   ```