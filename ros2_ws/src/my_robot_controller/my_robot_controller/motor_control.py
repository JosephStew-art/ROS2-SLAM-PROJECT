#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(Float32MultiArray, '/wheel_velocities', self.listener_callback, 10)

        # Define GPIO pins for motor control
        self.left_motor_forward = 17
        self.left_motor_backward = 27
        self.right_motor_forward = 5
        self.right_motor_backward = 6

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_motor_forward, GPIO.OUT)
        GPIO.setup(self.left_motor_backward, GPIO.OUT)
        GPIO.setup(self.right_motor_forward, GPIO.OUT)
        GPIO.setup(self.right_motor_backward, GPIO.OUT)

        # Set PWM frequency and initial duty cycle
        self.left_pwm_forward = GPIO.PWM(self.left_motor_forward, 100)
        self.left_pwm_backward = GPIO.PWM(self.left_motor_backward, 100)
        self.right_pwm_forward = GPIO.PWM(self.right_motor_forward, 100)
        self.right_pwm_backward = GPIO.PWM(self.right_motor_backward, 100)
        self.left_pwm_forward.start(0)
        self.left_pwm_backward.start(0)
        self.right_pwm_forward.start(0)
        self.right_pwm_backward.start(0)

    def listener_callback(self, msg):
        left_speed, right_speed = msg.data
        self.set_motor_speeds(left_speed, right_speed)

    def set_motor_speeds(self, left_speed, right_speed):
        # Set left motor speed
        if left_speed > 0:
            self.left_pwm_forward.ChangeDutyCycle(min(abs(left_speed), 100))
            self.left_pwm_backward.ChangeDutyCycle(0)
        else:
            self.left_pwm_forward.ChangeDutyCycle(0)
            self.left_pwm_backward.ChangeDutyCycle(min(abs(left_speed), 100))

        # Set right motor speed
        if right_speed > 0:
            self.right_pwm_forward.ChangeDutyCycle(min(abs(right_speed), 100))
            self.right_pwm_backward.ChangeDutyCycle(0)
        else:
            self.right_pwm_forward.ChangeDutyCycle(0)
            self.right_pwm_backward.ChangeDutyCycle(min(abs(right_speed), 100))

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
