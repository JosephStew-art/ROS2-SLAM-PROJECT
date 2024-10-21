# This is the documentation for setting up the ROS 2 workspaces on both the Raspberry Pi and the base station

## Raspberry Pi Workspace Setup

1. Create a a directory for the package
```markdown
mkdir -p ~/ros2_ws/src/my_robot_package
cd ~/ros2_ws/src/my_robot_package
```
2. Create the package structure:
```markdown
ros2 pkg create --build-type ament_python my_robot_package
```
3. Create the python ROS 2 nodes in the ~/ros2_ws/src/my_robot_package/my_robot_package/ directory
4. Update the setup.py file to link the python nodes
5. Update the package.xml to add dependencies for rclpy, std_msgs, geometry_msgs, sensor_msgs if not already present
6. Build the package
```markdown
cd ~/ros2_ws
colcon build --packages-select my_robot_package
```
7. Source the setup file
```markdown
source ~/ros2_ws/install/setup.bash
```
8. Add the setup file to the .bashrc
9. Add a launch directory
```markdown
mkdir -p ~/ros2_ws/src/ros2_robot_package/ros2_robot_package/launch
```
10. Create the python launch file
```markdown
touch ~/ros2_ws/src/ros2_robot_package/launch/robot_nodes.launch.py
```
11. Edit the launch file to start the nodes
12. Build the package
```markdown
cd ~/ros2_ws
colcon build --packages-select my_robot_package
```
13. Source the setup file
```markdown
source ~/ros2_ws/install/setup.bash
```

## Base Station Workspace Setup

