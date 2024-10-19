from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_robot_package',
            executable='encoder_publisher',
            name='encoder_publisher'
        ),
        Node(
            package='ros2_robot_package',
            executable='velocity_computation',
            name='velocity_computation'
        ),
        Node(
            package='ros2_robot_package',
            executable='motor_control',
            name='motor_control'
        ),
        Node(
            package='ros2_robot_package',
            executable='camera_publisher',
            name='camera_publisher'
        )
    ])