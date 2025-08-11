#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class YellowLineDetector(Node):
    def __init__(self):
        super().__init__('yellow_line_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/rear_camera/image_raw',  # 또는 '/camera/image_raw'
            self.image_callback,
            10
        )

        self.complete_pub = self.create_publisher(
            Bool,
            '/line_detect/complete',
            10
        )

        # 디버깅용 이미지 퍼블리셔
        self.mask_pub = self.create_publisher(Image, '/debug/mask_image', 10)
        self.roi_pub = self.create_publisher(Image, '/debug/roi_image', 10)
        self.ratio_srv = self.create_service(
            Trigger,
            'rear_camera/get_result',
            self.handle_get_yellow_ratio
        )
        self.get_logger().info("✅ Yellow line detector with debug image publisher initialized.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # ROI 설정 (하단 1/4)
        h, w = mask.shape
        roi = mask[int(h * 0.8):, :]

        self.yellow_ratio = cv2.countNonZero(roi) / roi.size
        self.get_logger().info(f"Yellow ratio in ROI: {self.yellow_ratio:.3f}")
    

        # 완료 판단
        if self.yellow_ratio > 0.6:
            msg = Bool()
            msg.data = True
            self.complete_pub.publish(msg)
            self.get_logger().info("✅ Line detected! Published complete = True.")
        else:
            msg = Bool()
            msg.data = False
            self.complete_pub.publish(msg)
            self.get_logger().info("Line not detected! Published complete = False.")

        # 디버깅 이미지 퍼블리시
        # mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        roi_colored = np.zeros_like(frame)
        roi_colored[int(h * 0.8):, :] = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        # roi_msg = self.bridge.cv2_to_imgmsg(roi_colored, encoding='bgr8')

        # self.mask_pub.publish(mask_msg)
        # self.roi_pub.publish(roi_msg)

    def handle_get_yellow_ratio(self, request, response):
            response.message = self.yellow_ratio
            return response
    
def main(args=None):
    rclpy.init(args=args)
    node = YellowLineDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
