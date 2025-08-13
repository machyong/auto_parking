#!/usr/bin/env python3
#################################################################################
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################
#
# Authors: Ryan Shim, Gilbert, ChanHyeong Lee

import math
import os
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from std_srvs.srv import Empty

from turtlebot3_msgs.srv import Dqn
from turtlebot3_msgs.srv import Goal

from std_msgs.msg import Float64
from gazebo_msgs.msg import ContactsState  # 맨 위 import 섹션에 추가


from std_msgs.msg import Float64, Bool

ROS_DISTRO = os.environ.get('ROS_DISTRO')


class RLEnvironment(Node):

    def __init__(self):
        super().__init__('rl_environment')
        self.goal_pose_x = 0.2648
        self.goal_pose_y = 1.1942
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0

        # self.action_size = 5
        self.action_size = 11
        self.max_step = 800

        self.done = False
        self.fail = False
        self.succeed = False
        self.parkingline_ratio = 0.0
        self.collisoin_flag = False
        self.goal_angle = 0.0
        self.goal_distance = 1.0
        self.init_goal_distance = 0.5
        self.scan_ranges = []
        self.front_ranges = []
        self.min_obstacle_distance = 10.0
        self.is_front_min_actual_front = False

        # 충돌센서 변수 추가
        self.bumper_in_contact = False
        self.bumper_force_norm = 0.0
        self.bumper_max_depth = 0.0


        self.local_step = 0
        self.stop_cmd_vel_timer = None
        # self.angular_vel = [1.5, 0.75, 0.0, -0.75, -1.5]
        self.angular_vel = [[0.1, 1.5], [0.1, 0.75], [0.1, 0.0], [0.1, -0.75], [0.1, -1.5],
                            [0.0, 0.0],
                            [-0.1, 1.5], [-0.1, 0.75], [-0.1, 0.0], [-0.1, -0.75], [-0.1, -1.5]]
        qos = QoSProfile(depth=10)

        if ROS_DISTRO == 'humble':
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_sub_callback,
            qos
        )
        
        ######################## 카메라 3종으로 교체  service client로
        # self.scan_sub = self.create_subscription(
        #     LaserScan,
        #     'scan',
        #     self.scan_sub_callback,
        #     qos_profile_sensor_data
        # )
        self.rear_yellow_ratio = self.create_subscription(
            Float64,
            '/rear/ratio',
            self.ratio_callback,
            10
        )

        self.collision_flag = self.create_subscription(
            Bool, 
            '/collision/flag',
            self.collision_flag_callback,
            10
        )

        self.clients_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.task_succeed_client = self.create_client(
            Goal,
            'task_succeed',
            callback_group=self.clients_callback_group
        )
        self.task_failed_client = self.create_client(
            Goal,
            'task_failed',
            callback_group=self.clients_callback_group
        )
        self.initialize_environment_client = self.create_client(
            Goal,
            'initialize_env',
            callback_group=self.clients_callback_group
        )
        
        self.rl_agent_interface_service = self.create_service(
            Dqn,
            'rl_agent_interface',
            self.rl_agent_interface_callback
        )
        self.make_environment_service = self.create_service(
            Empty,
            'make_environment',
            self.make_environment_callback
        )
        self.reset_environment_service = self.create_service(
            Dqn,
            'reset_environment',
            self.reset_environment_callback
        )

        # 충돌센서 토픽 추가
        self.bumper_sub = self.create_subscription(
            ContactsState,
            '/bumper_states',
            self.bumper_sub_callback,
            qos_profile_sensor_data  # 또는 QoSProfile(depth=10)
        )


    def ratio_callback(self,msg):
        self.parkingline_ratio = msg.data

    def collision_flag_callback(self,msg):
        self.collision_flag = msg.data

    def make_environment_callback(self, request, response):
        self.get_logger().info('Make environment called')
        while not self.initialize_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'service for initialize the environment is not available, waiting ...'
            )
        future = self.initialize_environment_client.call_async(Goal.Request())
        rclpy.spin_until_future_complete(self, future)
        response_goal = future.result()
        if not response_goal.success:
            self.get_logger().error('initialize environment request failed')
        else:
            self.goal_pose_x = response_goal.pose_x
            self.goal_pose_y = response_goal.pose_y
            self.get_logger().info(
                'goal initialized at [%f, %f]' % (self.goal_pose_x, self.goal_pose_y)
            )

        return response

    # dqn_agent에서 학습 끝나고 다음 에피소드를 시작할 때 호출
    def reset_environment_callback(self, request, response):
        state = self.calculate_state()
        self.init_goal_distance = state[0]
        self.prev_goal_distance = self.init_goal_distance #이것만 사용
        response.state = state

        return response

    # 목표에 도달했을 때 dqn_gazebo.py의 task_succeed 서비스로 요청 보내고 새 목표 좌표를 받아 저장.
    def call_task_succeed(self):
        while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for task succeed is not available, waiting ...')
        future = self.task_succeed_client.call_async(Goal.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.goal_pose_x = response.pose_x
            self.goal_pose_y = response.pose_y
            self.get_logger().info('service for task succeed finished')
        else:
            self.get_logger().error('task succeed service call failed')
    # 목표에 도달했을 때 dqn_gazebo.py의 task_failed 서비스로 요청 보내고 새 목표 좌표를 받아 저장.
    def call_task_failed(self):
        while not self.task_failed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for task failed is not available, waiting ...')
        future = self.task_failed_client.call_async(Goal.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.goal_pose_x = response.pose_x
            self.goal_pose_y = response.pose_y
            self.get_logger().info('service for task failed finished')
        else:
            self.get_logger().error('task failed service call failed')

    # odom 계산
    def odom_sub_callback(self, msg):
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        _, _, self.robot_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        goal_distance = math.sqrt(
            (self.goal_pose_x - self.robot_pose_x) ** 2
            + (self.goal_pose_y - self.robot_pose_y) ** 2)
        path_theta = math.atan2(
            self.goal_pose_y - self.robot_pose_y,
            self.goal_pose_x - self.robot_pose_x)

        goal_angle = path_theta - self.robot_pose_theta
        if goal_angle > math.pi:
            goal_angle -= 2 * math.pi

        elif goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = goal_distance
        self.goal_angle = goal_angle
        
    # 충돌 감지 콜백함수 추가
    def bumper_sub_callback(self, msg: ContactsState):
        # 기본: states가 비어있지 않으면 접촉 중
        in_contact = len(msg.states) > 0

        force_norm = 0.0
        max_depth = 0.0

        for st in msg.states:
            # total_wrench.force 크기(뉴턴)
            fx = st.total_wrench.force.x
            fy = st.total_wrench.force.y
            fz = st.total_wrench.force.z
            fn = (fx**2 + fy**2 + fz**2) ** 0.5
            force_norm = max(force_norm, fn)

            # depths가 비어있지 않으면 최대 관입 깊이 추출
            if st.depths:
                max_depth = max(max_depth, max(st.depths))

        # 간단 버전: states가 비어있지 않으면 True
        # 보수적 버전(노이즈 필터링): force_norm > 0.5 N 또는 max_depth > 1e-5 m
        CONTACT_FORCE_THRESHOLD = 0.5
        CONTACT_DEPTH_THRESHOLD = 1e-5

        self.bumper_in_contact = in_contact and (
            (force_norm > CONTACT_FORCE_THRESHOLD) or (max_depth > CONTACT_DEPTH_THRESHOLD)
        )
        self.bumper_force_norm = force_norm
        self.bumper_max_depth = max_depth

    def calculate_state(self):
        state = []
        state.append(float(self.goal_distance))
        state.append(float(self.parkingline_ratio))
        self.local_step += 1
        self.get_logger().info(f'current step : {self.local_step}')

        if self.goal_distance < 0.1:
            self.get_logger().info('Goal Reached')
            self.succeed = True
            self.done = True
            msg = Twist() 
            msg.linear.x = 0.
            msg.angular.z = 0.
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(msg)
            self.local_step = 0

            self.call_task_succeed()
        # self.min_obstacle_distanse 산출방식 수정 필요
        if self.bumper_in_contact:  # Contact Sensor 충돌 조
        if self.collision_flag == True:
            self.get_logger().info('Collision happened')
            self.fail = True
            self.done = True
            msg = Twist() 
            msg.linear.x = 0.
            msg.angular.z = 0.
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(msg)

            self.local_step = 0
            self.call_task_failed()

        # 시간초과
        if self.local_step == self.max_step:
            self.get_logger().info('Time out!')
            self.fail = True
            self.done = True
            msg = Twist() 
            msg.linear.x = 0.
            msg.angular.z = 0.
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(msg)
            # else:
            #     self.cmd_vel_pub.publish(TwistStamped())
            self.local_step = 0
            self.call_task_failed()

        return state

    def calculate_reward(self):
        reward = 1.0 - float(self.goal_distance)*2 - float(self.parkingline_ratio)*0.4 - float(self.local_step/self.max_step)*0.15
        if self.parkingline_ratio < 0.6:
            reward = 0.0
        return reward

    # 로봇 동작 수행
    def rl_agent_interface_callback(self, request, response):
        action = request.action
        if ROS_DISTRO == 'humble':
            msg = Twist() 
            msg.linear.x = self.angular_vel[action][0]
            msg.angular.z = self.angular_vel[action][1]
        self.cmd_vel_pub.publish(msg)

        if self.stop_cmd_vel_timer is None:
            self.prev_goal_distance = self.init_goal_distance
            self.stop_cmd_vel_timer = self.create_timer(0.3, self.timer_callback)
        else:
            self.destroy_timer(self.stop_cmd_vel_timer)
            self.stop_cmd_vel_timer = self.create_timer(0.3, self.timer_callback)

        response.state = self.calculate_state()
        response.reward = self.calculate_reward()
        response.done = self.done

        if self.done is True:
            self.done = False
            self.succeed = False
            self.fail = False

        return response

    def timer_callback(self):
        self.get_logger().info('Stop called')
        if ROS_DISTRO == 'humble':
            self.cmd_vel_pub.publish(Twist())
        # else:
        #     self.cmd_vel_pub.publish(TwistStamped())
        self.destroy_timer(self.stop_cmd_vel_timer)

    # odom x,y,z,w값을 방위각 값으로 변환
    def euler_from_quaternion(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    rl_environment = RLEnvironment()
    try:
        while rclpy.ok():
            rclpy.spin_once(rl_environment, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rl_environment.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
