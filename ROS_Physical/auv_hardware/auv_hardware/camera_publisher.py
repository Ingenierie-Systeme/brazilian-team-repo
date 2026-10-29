#!/usr/bin/env python3
# Copyright 2016 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0.
"""
ROS 2 Publisher node for Camera using OpenCV and cv_bridge.
Captures frames from USB / CSI camera and publishes to /camera/image_raw
and /camera/compressed_image.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

try:
    import cv2
    import numpy as np
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')

        # Parameters
        self.declare_parameter('device_id', 0)            # 0 for laptop webcam, 2 for USB camera
        self.declare_parameter('publish_compressed', True)

        self.device_id = self.get_parameter('device_id').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.rate = 30 # Frame rate in Hz
        self.width = 640
        self.height = 480
        self.frame_id = 'camera_link'
        self.quality = 50

        self.bridge = CvBridge()
        self.cap = None

        # Publishers
        self.raw_publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        if self.publish_compressed:
            self.compressed_publisher_ = self.create_publisher(
                CompressedImage, 'camera/image_raw/compressed', 10
            )


        # Initialize Camera using V4L2 directly
        self.init_camera()

        timer_period = 1.0 / max(0.1, float(self.rate))
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f'Camera publisher started at {self.rate} Hz on "camera/image_raw" '
            f'({self.width}x{self.height}, device={self.device_id})'
        )

    def init_camera(self):

        dev = int(self.device_id) if str(self.device_id).isdigit() else self.device_id

        # Use V4L2 backend directly on Linux to avoid GStreamer pipeline errors
        try:
            self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        except Exception:
            self.cap = cv2.VideoCapture(dev)

        

        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.rate)

        
        else:
            self.get_logger().warn(
                f'Could not open video device {self.device_id}. Running in mock test pattern mode.'
            )

    def timer_callback(self):
        frame = None
        now = self.get_clock().now().to_msg()

        # Capture from hardware
        if self.cap is not None and self.cap.isOpened():
            ret, captured = self.cap.read()
            if ret and captured is not None:
                frame = captured
            else:
                self.get_logger().warn_throttle(5.0, f'Failed to read frame from device {self.device_id}')


        if frame is None:
            return

        # 1. Publish Raw Image (sensor_msgs/Image) - works out-of-the-box in rqt_image_view
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = now
            img_msg.header.frame_id = self.frame_id
            self.raw_publisher_.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Error converting raw image: {e}')

        # 2. Publish Compressed Image (sensor_msgs/CompressedImage)
        if self.publish_compressed and HAS_OPENCV:
            try:
                comp_msg = CompressedImage()
                comp_msg.header.stamp = now
                comp_msg.header.frame_id = self.frame_id
                comp_msg.format = 'jpeg'
                success, encoded_img = cv2.imencode(
                    '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.quality)]
                )
                if success:
                    comp_msg.data = encoded_img.tobytes()
                    self.compressed_publisher_.publish(comp_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing compressed image: {e}')

    def release_camera(self):
        if self.cap is not None and self.cap.isOpened():
            self.get_logger().info('Releasing video device...')
            self.cap.release()


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        camera_publisher.release_camera()
        camera_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
