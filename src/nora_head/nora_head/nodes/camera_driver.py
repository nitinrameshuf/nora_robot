#!/usr/bin/env python3
# Copyright 2026 nanostation
# Licensed under the Apache License, Version 2.0

"""
Camera Driver Node for Nora
Captures video from 16MP USB camera and publishes to ROS 2
Uses system Python with JetPack OpenCV
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2


class CameraDriverNode(Node):
    def __init__(self):
        super().__init__('camera_driver')
        
        # Declare parameters
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('publish_rate', 15.0)
        
        # Get parameters
        self.device = self.get_parameter('device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Open camera
        self.cap = cv2.VideoCapture(self.device)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera: {self.device}')
            raise RuntimeError(f'Camera not found: {self.device}')
        
        # Configure camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.get_logger().info(f'Camera opened: {self.device}')
        self.get_logger().info(f'Resolution: {actual_width}x{actual_height} @ {actual_fps}fps')
        
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.frame_count = 0
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz')
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header = header
            self.image_pub.publish(image_msg)
            
            camera_info_msg = self.create_camera_info(header)
            self.camera_info_pub.publish(camera_info_msg)
            
            self.frame_count += 1
            
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')
        
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def create_camera_info(self, header):
        camera_info = CameraInfo()
        camera_info.header = header
        camera_info.width = self.width
        camera_info.height = self.height
        
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        fx = float(self.width)
        fy = float(self.height)
        cx = float(self.width / 2.0)
        cy = float(self.height / 2.0)
        
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        return camera_info
    
    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Camera released')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
