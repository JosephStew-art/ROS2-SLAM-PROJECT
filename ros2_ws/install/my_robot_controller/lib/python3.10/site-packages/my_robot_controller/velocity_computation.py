#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class VelocityComputationNode(Node):
    def __init__(self):
        super().__init__('velocity_computation_node')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.publisher = self.create_publisher(Float32MultiArray, '/wheel_velocities', 10)

    def listener_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Compute the motor commands
        left_speed, right_speed = self.compute_wheel_speeds(linear_velocity, angular_velocity)

        # Publish wheel speeds
        msg = Float32MultiArray()
        msg.data = [left_speed, right_speed]
        self.publisher.publish(msg)

    def compute_wheel_speeds(self, linear_velocity, angular_velocity):
        # Kinematic parameters
        wheel_base = 0.14  # distance between wheels
        wheel_radius = 0.03125  # radius of the wheels

        v_left = (2 * linear_velocity - angular_velocity * wheel_base) / (2 * wheel_radius)
        v_right = (2 * linear_velocity + angular_velocity * wheel_base) / (2 * wheel_radius)

        return v_left, v_right

def main(args=None):
    rclpy.init(args=args)
    node = VelocityComputationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
