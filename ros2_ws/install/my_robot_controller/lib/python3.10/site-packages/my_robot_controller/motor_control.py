#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(Float32MultiArray, '/wheel_velocities', self.listener_callback, 10)

        # GPIO pin setup
        self.gpio_pins_front = [17, 27]
        self.gpio_pins_back = [5, 6]

        self.gpio_pins_forward_right= 17
        self.gpio_pins_forward_left = 27
        self.gpio_pins_backwards_right = 5
        self.gpio_pins_backwards_left = 6

        # GPIO pin setup for PWM control
        self.pwm_pin_left = 18
        self.pwm_pin_right = 19

        # GPIO pins setup for PWM speed control
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in self.gpio_pins_front + self.gpio_pins_back:
            GPIO.setup(pin, GPIO.OUT)

        GPIO.setup(self.pwm_pin_left, GPIO.OUT)
        GPIO.setup(self.pwm_pin_right, GPIO.OUT)

        # Initialize PWM on the GPIO pins
        self.pwm_left = GPIO.PWM(self.pwm_pin_left, 100)  # 100 Hz frequency
        self.pwm_right = GPIO.PWM(self.pwm_pin_right, 100)

        self.pwm_left.start(0)
        self.pwm_right.start(0)

        # Set initial state to stop
        GPIO.output(self.gpio_pins_forward_right, GPIO.LOW)
        GPIO.output(self.gpio_pins_forward_left, GPIO.LOW)
        GPIO.output(self.gpio_pins_backwards_right, GPIO.LOW)
        GPIO.output(self.gpio_pins_backwards_left, GPIO.LOW)

    def listener_callback(self, msg):
        left_speed, right_speed = msg.data
        self.set_motor_speeds(left_speed, right_speed)

    def set_motor_speeds(self, left_speed, right_speed):
        # Set left motor direction
        if left_speed > 0:
            GPIO.output(self.gpio_pins_forward_left, GPIO.HIGH)
            GPIO.output(self.gpio_pins_backwards_left, GPIO.LOW)
        else:
            GPIO.output(self.gpio_pins_forward_left, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_left, GPIO.HIGH)

        # Set right motor direction
        if right_speed > 0:
            GPIO.output(self.gpio_pins_forward_right, GPIO.HIGH)
            GPIO.output(self.gpio_pins_backwards_right, GPIO.LOW)
        else:
            GPIO.output(self.gpio_pins_forward_right, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_right, GPIO.HIGH)

        # Set PWM duty cycle based on speed (0 to 100)
        self.pwm_left.ChangeDutyCycle(abs(left_speed))
        self.pwm_right.ChangeDutyCycle(abs(right_speed))

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
