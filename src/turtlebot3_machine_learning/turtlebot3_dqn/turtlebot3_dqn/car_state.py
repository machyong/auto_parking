#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


class GoalStatePublisher(Node):
    def __init__(self):
        super().__init__('goal_state_publisher')

        # 목표 좌표 (원하는 값으로 수정 가능)
        self.goal_x = 0.25
        self.goal_y = 1.1905

        # 퍼블리셔: [거리, yaw 오차] 배열로 발행
        self.state_pub = self.create_publisher(Float32MultiArray, '/goal_state', 10)

        # 구독: 로봇 위치/자세
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info("✅ Goal State Publisher Node Started.")

    def odom_callback(self, msg: Odometry):
        # 로봇 현재 위치
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 로봇 현재 yaw
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

        # 목표까지의 거리 계산
        dx = self.goal_x - x
        dy = self.goal_y - y
        distance = math.sqrt(dx * dx + dy * dy)

        # 목표 방향 (로봇→목표)
        path_theta = math.atan2(dy, dx)

        # yaw 오차 (목표 yaw - 현재 yaw)
        yaw_error = path_theta - yaw
        if yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        elif yaw_error < -math.pi:
            yaw_error += 2 * math.pi

        # 퍼블리시
        msg_out = Float32MultiArray()
        msg_out.data = [distance, yaw_error]
        self.state_pub.publish(msg_out)

        # 🔹 로그로 출력
        self.get_logger().info(
            f"[Goal State] Distance = {distance:.3f} m | Yaw Error = {yaw_error:.3f} rad ({math.degrees(yaw_error):.2f}°)"
        )

    def euler_from_quaternion(self, quat):
        """geometry_msgs/msg/Quaternion → roll, pitch, yaw 변환"""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = GoalStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
