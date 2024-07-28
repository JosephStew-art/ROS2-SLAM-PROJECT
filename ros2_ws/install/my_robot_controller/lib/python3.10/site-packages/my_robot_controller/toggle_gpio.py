#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import socket

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # GPIO pin setup
        self.gpio_pins_1 = [17, 27]
        self.gpio_pins_2 = [5, 6]

        self.gpio_pins_1_1 = 17
        self.gpio_pins_1_2 = 27
        self.gpio_pins_2_1 = 5
        self.gpio_pins_2_2 = 6


        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in self.gpio_pins_1 + self.gpio_pins_2:
            GPIO.setup(pin, GPIO.OUT)

        # Set initial state to stop
        self.stop()

        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 12345))

        # Create a timer to check for incoming UDP messages
        self.timer = self.create_timer(0.1, self.check_udp)

    def check_udp(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            command = data.decode('utf-8')
            self.get_logger().info(f'Received command: {command}')
            self.command_callback(command)
        except socket.error as e:
            self.get_logger().error(f'Socket error: {e}')


    def command_callback(self, command):
        if command == 'w':
            self.forward()
        elif command == 's':
            self.backward()
        elif command == 'a':
            self.rotate_counterclockwise()
        elif command == 'd':
            self.rotate_clockwise()
        elif command == 'x':
            self.stop()

    def forward(self):
        GPIO.output(self.gpio_pins_1_1, GPIO.HIGH)
        GPIO.output(self.gpio_pins_1_2, GPIO.HIGH)
        GPIO.output(self.gpio_pins_2_1, GPIO.LOW)
        GPIO.output(self.gpio_pins_2_2, GPIO.LOW)

    def backward(self):
        GPIO.output(self.gpio_pins_1_1, GPIO.LOW)
        GPIO.output(self.gpio_pins_1_2, GPIO.LOW)
        GPIO.output(self.gpio_pins_2_1, GPIO.HIGH)
        GPIO.output(self.gpio_pins_2_2, GPIO.HIGH)

    def rotate_clockwise(self):
        GPIO.output(self.gpio_pins_1_1, GPIO.LOW)
        GPIO.output(self.gpio_pins_1_2, GPIO.HIGH)
        GPIO.output(self.gpio_pins_2_1, GPIO.HIGH)
        GPIO.output(self.gpio_pins_2_2, GPIO.LOW)

    def rotate_counterclockwise(self):
        GPIO.output(self.gpio_pins_1_1, GPIO.HIGH)
        GPIO.output(self.gpio_pins_1_2, GPIO.LOW)
        GPIO.output(self.gpio_pins_2_1, GPIO.LOW)
        GPIO.output(self.gpio_pins_2_2, GPIO.HIGH)

    def stop(self):
        GPIO.output(self.gpio_pins_1, GPIO.LOW)
        GPIO.output(self.gpio_pins_2, GPIO.LOW)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
