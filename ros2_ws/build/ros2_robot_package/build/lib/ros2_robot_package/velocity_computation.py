#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class VelocityComputationNode(Node):
    def __init__(self):
        super().__init__('velocity_computation_node')
        self.publisher = self.create_publisher(Float32MultiArray, '/wheel_velocities', 10)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Robot parameters
        self.wheel_base = 0.14  # distance between wheels in meters
        self.wheel_radius = 0.03125  # radius of the wheels in meters
        
        # Predetermined speeds
        self.linear_speed = 0.25  # m/s
        self.angular_speed = 2.5  # rad/s

    def cmd_vel_callback(self, msg):
        # Interpret normalized cmd_vel and apply predetermined speeds
        if msg.linear.x != 0:
            linear_velocity = self.linear_speed * msg.linear.x
            angular_velocity = 0.0
        elif msg.angular.z != 0:
            linear_velocity = 0.0
            angular_velocity = self.angular_speed * msg.angular.z
        else:
            linear_velocity = 0.0
            angular_velocity = 0.0
        
        left_speed, right_speed = self.compute_wheel_speeds(linear_velocity, angular_velocity)
        self.publish_wheel_speeds(left_speed, right_speed)

    def compute_wheel_speeds(self, linear_velocity, angular_velocity):
        v_left = (2 * linear_velocity - angular_velocity * self.wheel_base) / (2 * self.wheel_radius)
        v_right = (2 * linear_velocity + angular_velocity * self.wheel_base) / (2 * self.wheel_radius)
        return v_left, v_right

    def publish_wheel_speeds(self, left_speed, right_speed):
        msg = Float32MultiArray()
        msg.data = [left_speed, right_speed]
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing wheel speeds: Left: {left_speed}, Right: {right_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityComputationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()