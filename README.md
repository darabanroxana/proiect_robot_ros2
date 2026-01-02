# turtlebot3_football ROS 2 workspace

This workspace contains a simple ROS 2 Python package `turtlebot3_football`.

Files created:

- `src/turtlebot3_football/package.xml` - package manifest with dependencies
- `src/turtlebot3_football/setup.py` - setuptools install script
- `src/turtlebot3_football/resource/turtlebot3_football` - ament resource marker
- `src/turtlebot3_football/turtlebot3_football/vision_node.py` - example node

Terminal commands

1) If you prefer to create the package using ROS 2 tooling (alternative to the files here):

```bash
cd /home/roxana/Desktop/proiect_robot/src
ros2 pkg create --build-type ament_python --dependencies rclpy geometry_msgs sensor_msgs cv_bridge turtlebot3_football
```

2) Build the workspace with colcon (from workspace root):

```bash
cd /home/roxana/Desktop/proiect_robot
source /opt/ros/<ros2-distro>/setup.bash   # replace <ros2-distro> with e.g. humble, iron, etc.
colcon build --symlink-install
```

3) Source the local install and run the example node:

```bash
source install/setup.bash
ros2 run turtlebot3_football vision_node
```

Notes
- The package declares `cv_bridge` and lists `python3-opencv` as an exec_depend in `package.xml`. Install OpenCV in the Python environment (e.g., `pip install opencv-python`) or the system package (`sudo apt install python3-opencv`).
- Edit `maintainer` and emails in `package.xml` and `setup.py` as appropriate.
