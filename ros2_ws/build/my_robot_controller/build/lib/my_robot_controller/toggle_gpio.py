#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

class GPIOToggleNode(Node):
    def __init__(self):
        super().__init__('gpio_toggle_node')
        self.get_logger().info('GPIO Toggle Node has started.')
        
        # Set the GPIO mode
        GPIO.setmode(GPIO.BCM)
        
        # Define GPIO pins
        self.pin1 = 17  # GPIO17 (Pin 11)
        self.pin2 = 27  # GPIO27 (Pin 13)
        self.pin3 = 5   # GPIO5 (Pin 29)
        self.pin4 = 6   # GPIO6 (Pin 31)
        
        # Set pins as output
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)
        GPIO.setup(self.pin3, GPIO.OUT)
        GPIO.setup(self.pin4, GPIO.OUT)
        
        # Start the timer to toggle GPIO pins every 2 seconds
        self.timer = self.create_timer(2.0, self.toggle_pins)
        
        # Initial state
        self.state = False

    def toggle_pins(self):
        self.state = not self.state
        # Toggle the state of pin1 and pin2
        GPIO.output(self.pin1, self.state)
        GPIO.output(self.pin2, self.state)
        
        # Set pin3 and pin4 to the opposite state of pin1 and pin2
        GPIO.output(self.pin3, not self.state)
        GPIO.output(self.pin4, not self.state)
        
        state_str = 'HIGH' if self.state else 'LOW'
        self.get_logger().info(f'GPIO17 and GPIO27 set to {state_str}')
        self.get_logger().info(f'GPIO5 and GPIO6 set to {"LOW" if self.state else "HIGH"}')

def main(args=None):
    rclpy.init(args=args)
    node = GPIOToggleNode()
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
