# Mobile Robotics course @ UNIVR
_Unity simulation of TurtleBot3 Waffle Pi and RB-KAIROS robots over ROS2._

A report of this project can be found [here](https://www.overleaf.com/read/qfqdpnxpnphg).

To test the project, move in the desired `ros2` subfolder (either `kairos` or `turtlebot3`) and open the related Unity project. Then, simply run
- bringup command: `ros2 launch kairos_bringup.py` (or `turtlebot3_bringup.py`)
- teleop command: `python3 kairos_teleop.py` (or `turtlebot3_teleop.py`)
- Unity command: `Play button`
- when satisfied with the result, map saver command: `ros2 run nav2_map_server map_saver_cli -f ./icelab_map --ros-args -p save_map_timeout:=10000`

To shutdown the setup, simply terminate the processes following the list in reverse. As an example, the map obtained in the Kairos setup is shown below

<p align="center">
  <img src="doc/icelab_map.png" alt="ICE Lab map obtained from Kairos SLAM" />
</p>
