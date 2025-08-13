import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class ParkingZoneDetector(Node):
    def __init__(self):
        super().__init__('parking_zone_detector')
        self.subscription = self.create_subscription(
            Image,
            '/right_camera/image_raw',
            self.image_callback,
            10)
        self.bool_pub = self.create_publisher(
            Bool,
            '/parking_zone_detected',
            10)
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_img.shape

        roi_start = int(h * 0.2)
        roi = cv_img[roi_start:, :]
        roi_area = roi.shape[0] * roi.shape[1]  # ROI 전체 면적

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv, (15, 100, 100), (35, 255, 255))
        mask_white = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))
        mask_all = cv2.bitwise_or(mask_yellow, mask_white)

        mask_inv = cv2.bitwise_not(mask_all)
        contours, _ = cv2.findContours(mask_inv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        parking_zone_found = False

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            area = cv2.contourArea(cnt)
            # 넓이가 ROI의 40% 이상인 사각형만 주차구역으로 인정
            if len(approx) == 4 and area > 2000 and area > roi_area * 0.4:
                parking_zone_found = True
                # cv2.drawContours(roi, [approx], 0, (0, 255, 0), 2)
                # cv2.putText(roi, "parking area", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                break

        # if not parking_zone_found:
        #     cv2.putText(roi, "Looking for it", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

        # 퍼블리시 (Bool)
        bool_msg = Bool()
        bool_msg.data = parking_zone_found
        if parking_zone_found == True:
            self.bool_pub.publish(bool_msg)

        # cv2.imshow("Parking Zone Result", roi)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingZoneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
