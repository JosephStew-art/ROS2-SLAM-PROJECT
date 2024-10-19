#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CompressedCameraPublisher(Node):
    def __init__(self):
        super().__init__('compressed_camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)
        self.timer = self.create_timer(0.033333, self.timer_callback)  # 10 Hz
        self.cv_bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Open the default camera (usually the webcam)
        
        # Set camera parameters (adjust as needed)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()  # Changed from tostring() to tobytes()
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    compressed_camera_publisher = CompressedCameraPublisher()
    rclpy.spin(compressed_camera_publisher)
    compressed_camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()