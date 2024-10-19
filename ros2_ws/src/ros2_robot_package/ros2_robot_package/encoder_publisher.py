#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)
        
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        self.LEFT_ENCODER_PIN = 23  # Change to your actual GPIO pin
        self.RIGHT_ENCODER_PIN = 24  # Change to your actual GPIO pin
        GPIO.setup(self.LEFT_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.RIGHT_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Encoder tick counters and previous states
        self.left_ticks = 0
        self.right_ticks = 0
        self.left_state = GPIO.input(self.LEFT_ENCODER_PIN)
        self.right_state = GPIO.input(self.RIGHT_ENCODER_PIN)

        # Debouncing variables
        self.DEBOUNCE_THRESHOLD = 5
        self.left_debounce_count = 0
        self.right_debounce_count = 0

        # Set up timer for polling
        self.polling_rate = 0.001  # 1000 Hz polling rate
        self.create_timer(self.polling_rate, self.poll_encoders)

        # Set up timer for publishing
        self.publishing_rate = 0.001 # 1000 Hz publishing rate
        self.create_timer(self.publishing_rate, self.publish_encoder_ticks)

    def poll_encoders(self):

        # Check left encoder
        current_left_state = GPIO.input(self.LEFT_ENCODER_PIN)
        if current_left_state != self.left_state:
            self.left_debounce_count += 1
            if self.left_debounce_count >= self.DEBOUNCE_THRESHOLD:
                self.left_state = current_left_state
                self.left_debounce_count = 0
                if self.left_state == 1:
                    self.left_ticks += 1
        else:
            self.left_debounce_count = 0

        # Check right encoder
        current_right_state = GPIO.input(self.RIGHT_ENCODER_PIN)
        if current_right_state != self.right_state:
            self.right_debounce_count += 1
            if self.right_debounce_count >= self.DEBOUNCE_THRESHOLD:
                self.right_state = current_right_state
                self.right_debounce_count = 0
                if self.right_state == 1:
                    self.right_ticks += 1
        else:
            self.right_debounce_count = 0

    def publish_encoder_ticks(self):
        msg = Int32MultiArray()
        msg.data = [self.left_ticks, self.right_ticks]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    encoder_publisher = EncoderPublisher()
    try:
        rclpy.spin(encoder_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        encoder_publisher.destroy_node()
        GPIO.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()