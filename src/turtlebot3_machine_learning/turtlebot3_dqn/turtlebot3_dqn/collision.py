#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool, Float32

class CollisionMonitor(Node):
    def __init__(self):
        super().__init__('collision_monitor')
        self.sub = self.create_subscription(
            ContactsState, '/bumper_states', self.cb, 10
        )
        self.pub_flag = self.create_publisher(Bool, '/collision/flag', 10)

        # 간단한 디바운스/임계값
        self.min_force_N = 1.5     # 추천 시작값 (아래 설명)
        self.min_frames = 2        # 연속 N프레임 이상일 때만 true
        self.hit_streak = 0

    def cb(self, msg: ContactsState):
        # metric: 접촉들 중 최대 힘 크기(N)
        max_f = 0.0
        for s in msg.states:
            f = s.total_wrench.force
            mag = math.sqrt(f.x*f.x + f.y*f.y + f.z*f.z)
            if mag > max_f:
                max_f = mag

        # 판정 로직 (둘 중 하나 충족)
        # - states가 존재하고 (기본적으로 base는 바닥과 스치지 않음)
        # - 힘이 min_force_N 이상
        contact_present = len(msg.states) > 0
        hit_now = contact_present and (max_f >= self.min_force_N)

        # 디바운스(연속 프레임 확인)
        self.hit_streak = self.hit_streak + 1 if hit_now else 0
        collided = self.hit_streak >= self.min_frames

        # publish
        self.pub_flag.publish(Bool(data=collided))

        if collided:
            self.get_logger().info(f'Collision! force_max={max_f:.2f}N')

def main():
    rclpy.init()
    node = CollisionMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
