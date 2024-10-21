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

1. Create a new directory called "orbslam3" in the root directory of the github project
2. Add the remote control python node to this directory
3. Make the python file executable
```markdown
chmod +x colcon_ws/src/orbslam3_ros2/orbslam3/robot_remote_control.py
```
4. update the CMakeLists.txt file in the orbslam3_ros2 directory
```markdown
# Add this at the top
ament_python_install_package(${PROJECT_NAME})


# Install Python executables
install(PROGRAMS
  src/camera_publisher.py
  src/resolution_checker.py
  src/stream_test.py
  src/url_camera_publisher.py
  src/robot_remote_control.py
  DESTINATION lib/${PROJECT_NAME}
)
```
5. Update the package.xml file
```markdown
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>python3-pynput</depend>
```
6. Build the package
```markdown
cd colcon_ws
colcon build --packages-select orbslam3_ros2
```
7. Source the workspace
```markdown
source install/setup.bash
```
8. Make a launch directory and file
```markdown
mkdir -p colcon_ws/src/orbslam3_ros2/launch
touch colcon_ws/src/orbslam3_ros2/launch/slam_and_control.launch.py
```
9. Edit the launch file to launch the remote control node and the SLAM node, make sure to include the input arguments for the SLAM node when initialising it
10. Add the launch file to the CMakeLists.txt file
```markdown
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```
10. Build the package
```markdown
cd colcon_ws
colcon build --packages-select orbslam3_ros2
```
11. Source the workspace
```markdown
source install/setup.bash
```




