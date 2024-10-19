#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import RPi.GPIO as GPIO
import time

class SimpleMotorControlNode(Node):
    def __init__(self):
        super().__init__('simple_motor_control_node')
        self.subscription = self.create_subscription(
            Float32MultiArray, 
            '/wheel_velocities', 
            self.velocity_callback, 
            10
        )
        self.encoder_subscription = self.create_subscription(
            Int32MultiArray, 
            'encoder_ticks', 
            self.encoder_callback, 
            10
        )

        # GPIO pin setup
        self.gpio_pins_forward_right = 17
        self.gpio_pins_forward_left = 27
        self.gpio_pins_backwards_right = 5
        self.gpio_pins_backwards_left = 6

        # GPIO pin setup for PWM control
        self.pwm_pin_left = 18
        self.pwm_pin_right = 19

        # GPIO pins setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in [self.gpio_pins_forward_right, self.gpio_pins_forward_left, 
                    self.gpio_pins_backwards_right, self.gpio_pins_backwards_left,
                    self.pwm_pin_left, self.pwm_pin_right]:
            GPIO.setup(pin, GPIO.OUT)

        # Initialize PWM on the GPIO pins
        self.pwm_left = GPIO.PWM(self.pwm_pin_left, 100)  # 100 Hz frequency
        self.pwm_right = GPIO.PWM(self.pwm_pin_right, 100)

        self.pwm_left.start(0)
        self.pwm_right.start(0)

        # Set initial state to stop
        self.stop_motors()

        # Variables to track wheel ticks and time
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = self.get_clock().now()

        # Desired velocities
        self.desired_left_velocity = 0.0
        self.desired_right_velocity = 0.0

        # Control loop timer
        self.control_loop_rate = 0.1 # 10 Hz
        self.create_timer(self.control_loop_rate, self.control_loop)

        # Base PWM value for forward motion
        self.base_pwm = 7 # Adjust this value based on your robot's characteristics

        # Correction factors
        self.course_correction_factor = 0.8 # Adjust this value to change how aggressively the robot corrects its course
        self.speed_correction_factor = 0.1  # Adjust this value to change how aggressively the robot corrects its speed

        # Target tick rate (ticks per second) for desired speed
        self.target_tick_rate = 1  # Adjust this value based on your desired speed and encoder resolution

        # New variables for tick difference reset
        self.tick_difference_offset = 0
        self.prev_movement_state = False  # False for not moving straight, True for moving straight

    def velocity_callback(self, msg):
        new_left_velocity, new_right_velocity = msg.data
        
        # Check if we're transitioning to straight forward movement
        current_movement_state = self.should_move_straight(new_left_velocity, new_right_velocity)
        if current_movement_state and not self.prev_movement_state:
            # Reset the tick difference offset
            self.tick_difference_offset = self.left_ticks - self.right_ticks
            self.get_logger().info(f'Resetting tick difference offset to {self.tick_difference_offset}')

        self.desired_left_velocity, self.desired_right_velocity = new_left_velocity, new_right_velocity
        self.prev_movement_state = current_movement_state
        
        self.get_logger().info(f'Desired velocities: Left: {self.desired_left_velocity:.2f}, Right: {self.desired_right_velocity:.2f}')

    def encoder_callback(self, msg):
        self.left_ticks, self.right_ticks = msg.data

    def control_loop(self):
        if self.should_move_straight(self.desired_left_velocity, self.desired_right_velocity):
            self.move_straight_with_speed_control()
        else:
            self.set_motor_speeds(self.desired_left_velocity, self.desired_right_velocity)

    def should_move_straight(self, left_velocity, right_velocity):
        # Check if both desired velocities are positive and equal (within a small tolerance)
        return (left_velocity > 0 and 
                right_velocity > 0 and 
                abs(left_velocity - right_velocity) < 1e-6)

    def move_straight_with_speed_control(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds

        # Calculate current tick rates
        left_tick_rate = (self.left_ticks - self.prev_left_ticks) / dt
        right_tick_rate = (self.right_ticks - self.prev_right_ticks) / dt

        # Calculate average tick rate for speed control
        avg_tick_rate = (left_tick_rate + right_tick_rate) / 2

        # Calculate adjusted tick difference for course correction
        adjusted_tick_difference = (self.left_ticks - self.right_ticks) - self.tick_difference_offset

        # Calculate speed correction
        speed_error = self.target_tick_rate - avg_tick_rate
        speed_correction = speed_error * self.speed_correction_factor

        # Calculate course correction
        course_correction = adjusted_tick_difference * self.course_correction_factor

        # Apply corrections to PWM values
        left_pwm = max(0, min(100, self.base_pwm + speed_correction - course_correction))
        right_pwm = max(0, min(100, self.base_pwm + speed_correction + course_correction))

        # Set motor speeds
        self.set_motor_speeds(left_pwm, right_pwm)

        # Update previous values for next iteration
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.prev_time = current_time

    def set_motor_speeds(self, left_speed, right_speed):
        # Set left motor direction and speed
        if left_speed > 0:
            GPIO.output(self.gpio_pins_forward_left, GPIO.HIGH)
            GPIO.output(self.gpio_pins_backwards_left, GPIO.LOW)
            self.pwm_left.ChangeDutyCycle(min(abs(left_speed), 100))
        elif left_speed < 0:
            GPIO.output(self.gpio_pins_forward_left, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_left, GPIO.HIGH)
            self.pwm_left.ChangeDutyCycle(min(abs(left_speed), 100))
        else:
            GPIO.output(self.gpio_pins_forward_left, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_left, GPIO.LOW)
            self.pwm_left.ChangeDutyCycle(0)

        # Set right motor direction and speed
        if right_speed > 0:
            GPIO.output(self.gpio_pins_forward_right, GPIO.HIGH)
            GPIO.output(self.gpio_pins_backwards_right, GPIO.LOW)
            self.pwm_right.ChangeDutyCycle(min(abs(right_speed), 100))
        elif right_speed < 0:
            GPIO.output(self.gpio_pins_forward_right, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_right, GPIO.HIGH)
            self.pwm_right.ChangeDutyCycle(min(abs(right_speed), 100))
        else:
            GPIO.output(self.gpio_pins_forward_right, GPIO.LOW)
            GPIO.output(self.gpio_pins_backwards_right, GPIO.LOW)
            self.pwm_right.ChangeDutyCycle(0)

    def stop_motors(self):
        GPIO.output(self.gpio_pins_forward_right, GPIO.LOW)
        GPIO.output(self.gpio_pins_forward_left, GPIO.LOW)
        GPIO.output(self.gpio_pins_backwards_right, GPIO.LOW)
        GPIO.output(self.gpio_pins_backwards_left, GPIO.LOW)
        self.pwm_left.ChangeDutyCycle(0)
        self.pwm_right.ChangeDutyCycle(0)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()