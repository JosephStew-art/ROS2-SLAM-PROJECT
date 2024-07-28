#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import sleep
import RPi.GPIO as GPIO

class GPIOToggleNode(Node):
    def __init__(self):
        super().__init__('gpio_toggle_node')
        
        # GPIO pin setup
        self.gpio_pins_1 = [17, 27]
        self.gpio_pins_2 = [5, 6]
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        for pin in self.gpio_pins_1 + self.gpio_pins_2:
            GPIO.setup(pin, GPIO.OUT)

        self.state = False

        # Set initial state
        self.set_gpio_state(self.state)

        # Create a timer to toggle the GPIO pins
        self.timer = self.create_timer(2.0, self.toggle_gpio)

    def set_gpio_state(self, state):
        if state:
            GPIO.output(self.gpio_pins_1, GPIO.HIGH)
            GPIO.output(self.gpio_pins_2, GPIO.LOW)
        else:
            GPIO.output(self.gpio_pins_1, GPIO.LOW)
            GPIO.output(self.gpio_pins_2, GPIO.HIGH)

    def toggle_gpio(self):
        # Set both sets of GPIO pins to LOW for 1 second
        GPIO.output(self.gpio_pins_1, GPIO.LOW)
        GPIO.output(self.gpio_pins_2, GPIO.LOW)
        sleep(1)

        # Toggle state
        self.state = not self.state

        # Set GPIO pins based on the new state
        self.set_gpio_state(self.state)
        self.get_logger().info(f'State: {"HIGH" if self.state else "LOW"}')

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
