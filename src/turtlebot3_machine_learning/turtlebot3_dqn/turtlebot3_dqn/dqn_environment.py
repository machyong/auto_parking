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
from std_srvs.srv import Trigger

ROS_DISTRO = os.environ.get('ROS_DISTRO')


class RLEnvironment(Node):

    def __init__(self):
        super().__init__('rl_environment')
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0

        # self.action_size = 5
        self.action_size = 15
        self.max_step = 800

        self.done = False
        self.fail = False
        self.succeed = False

        self.goal_angle = 0.0
        self.goal_distance = 1.0
        self.linear_x = 0.0
        self.init_goal_distance = 0.5
        self.scan_ranges = []
        self.front_ranges = []
        self.min_obstacle_distance = 10.0
        self.is_front_min_actual_front = False


        self.local_step = 0
        self.stop_cmd_vel_timer = None
        # self.angular_vel = [1.5, 0.75, 0.0, -0.75, -1.5]
        self.angular_vel = [[0.3, 1.5], [0.3, 0.75], [0.3, 0.0], [0.3, -0.75], [0.3, -1.5],
                            [0.0, 1.5], [0.0, 0.75], [0.0, 0.0], [0.0, -0.75], [0.0, -1.5],
                            [-0.3, 1.5], [-0.3, 0.75], [-0.3, 0.0], [-0.3, -0.75], [-0.3, -1.5]]
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
        self.rear_cam_result_client = self.create_client(
            Trigger,
            'rear_camera/get_result',
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
    def query_rear_camera_result(self):
        while not self.rear_cam_result_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(f"service 'rear_camera/get_result' not available, waiting ...")

        req = Trigger.Request()

        future = self.rear_cam_result_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f"rear_camera call failed: {future.exception()}")
            return None

<<<<<<< Updated upstream
    # initialize_environment_client 실행
=======
        res = future.result()
        value = float(res.message.strip())
        return value
        
>>>>>>> Stashed changes
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
    
    '''라이더 데이터 처리'''
    # def scan_sub_callback(self, scan):
    #     self.scan_ranges = []
    #     self.front_ranges = []
    #     self.front_angles = []

<<<<<<< Updated upstream
=======
## 카메라 받아서 값 송신 사이드 카메라 service로 agent 시작용 , 후방 카메라 종료후 요청
    # def scan_sub_callback(self, scan):
    #     self.scan_ranges = []
    #     self.front_ranges = []
    #     self.front_angles = []

>>>>>>> Stashed changes
    #     num_of_lidar_rays = len(scan.ranges)
    #     angle_min = scan.angle_min
    #     angle_increment = scan.angle_increment

    #     self.front_distance = scan.ranges[0]

    #     for i in range(num_of_lidar_rays):
    #         angle = angle_min + i * angle_increment
    #         distance = scan.ranges[i]

    #         if distance == float('Inf'):
    #             distance = 3.5
    #         elif numpy.isnan(distance):
    #             distance = 0.0

    #         self.scan_ranges.append(distance)

    #         if (0 <= angle <= math.pi/2) or (3*math.pi/2 <= angle <= 2*math.pi):
    #             self.front_ranges.append(distance)
    #             self.front_angles.append(angle)

    #     self.min_obstacle_distance = min(self.scan_ranges)
    #     self.front_min_obstacle_distance = min(self.front_ranges) if self.front_ranges else 10.0
<<<<<<< Updated upstream
=======
    # 충돌 감지 함수 필요
>>>>>>> Stashed changes

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

        odom_linear_speed = msg.twist.twist.linear.x

        # 최근에 보낸 cmd_vel 명령 저장 변수 필요
        # 예: self.last_cmd_linear_speed 가 rl_agent_interface_callback에서 저장되도록
        speed_diff = abs(self.linear_x - odom_linear_speed)

        # 일정 기준 이상 속도 차이가 크면 충돌로 판단
        if self.linear_x > 0.05 and odom_linear_speed < 0.01 and speed_diff > 0.05:
            self.collision_detected = True
        else:
            self.collision_detected = False

    def calculate_state(self):
        state = []
        state.append(float(self.goal_distance))
        state.append(float(self.goal_angle))
<<<<<<< Updated upstream
        #front_ranges 산출방식 수정 필요
        for var in self.front_ranges:
            state.append(float(var))
=======
        # for var in self.front_ranges: 스캔값 변경
        #     state.append(float(var))
>>>>>>> Stashed changes
        self.local_step += 1

        if self.goal_distance < 0.20:
            self.get_logger().info('Goal Reached')
            self.succeed = True
            self.done = True
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(Twist())
            self.local_step = 0

            self.last_rear_result = self.query_rear_camera_result() ######################################################

            self.call_task_succeed()
<<<<<<< Updated upstream
        # self.min_obstacle_distanse 산출방식 수정 필요
        if self.collision_detected == True:
=======

        if self.min_obstacle_distance < 0.15: # 조건을 odom과 cmdvel 비교로 변경
>>>>>>> Stashed changes
            self.get_logger().info('Collision happened')
            self.fail = True
            self.done = True
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(Twist())
            self.local_step = 0
            self.call_task_failed()

<<<<<<< Updated upstream
        # 시간초과
        if self.local_step == self.max_step:
=======
        if self.local_step == self.max_step: # 최대치 지정
>>>>>>> Stashed changes
            self.get_logger().info('Time out!')
            self.fail = True
            self.done = True
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(Twist())
            # else:
            #     self.cmd_vel_pub.publish(TwistStamped())
            self.local_step = 0
            self.call_task_failed()

        return state

<<<<<<< Updated upstream
    #라이더 사용해서 각도 가중치 부여
    def compute_directional_weights(self, relative_angles, max_weight=10.0):
        power = 6
        raw_weights = (numpy.cos(relative_angles))**power + 0.1
        scaled_weights = raw_weights * (max_weight / numpy.max(raw_weights))
        normalized_weights = scaled_weights / numpy.sum(scaled_weights)
        return normalized_weights

    # 전방에 장애물과의 거리에 따라 가중치 부여 사용 X
    def compute_weighted_obstacle_reward(self):
        if not self.front_ranges or not self.front_angles:
            return 0.0

        front_ranges = numpy.array(self.front_ranges)
        front_angles = numpy.array(self.front_angles)

        valid_mask = front_ranges <= 0.5
        if not numpy.any(valid_mask):
            return 0.0

        front_ranges = front_ranges[valid_mask]
        front_angles = front_angles[valid_mask]

        relative_angles = numpy.unwrap(front_angles)
        relative_angles[relative_angles > numpy.pi] -= 2 * numpy.pi

        weights = self.compute_directional_weights(relative_angles, max_weight=10.0)

        safe_dists = numpy.clip(front_ranges - 0.25, 1e-2, 3.5)
        decay = numpy.exp(-3.0 * safe_dists)

        weighted_decay = numpy.dot(weights, decay)

        reward = - (1.0 + 4.0 * weighted_decay)

        return reward

    def calculate_reward(self):
        # 방향 보상 계산(목표 방향과 로봇 방향의 각도에 따른 계산)
        yaw_reward = 1 - (2 * abs(self.goal_angle) / math.pi)
        # 전방장애물 사용 X
        obstacle_reward = self.compute_weighted_obstacle_reward()

        print('directional_reward: %f, obstacle_reward: %f' % (yaw_reward, obstacle_reward))
        reward = yaw_reward + obstacle_reward

=======
    def calculate_reward(self):
    
        
        reward = float(self.goal_distance) * self.last_rear_result
>>>>>>> Stashed changes
        if self.succeed:
            reward = 100.0
        elif self.fail:
            reward = -50.0

        return reward

    # 로봇 동작 수행
    def rl_agent_interface_callback(self, request, response):
        action = request.action
        if ROS_DISTRO == 'humble':
<<<<<<< Updated upstream
            msg = Twist()
            msg.linear.x = [action[0]]
            msg.angular.z = self.angular_vel[action[1]]
            self.linear_x = [action[0]]
=======
            msg = Twist() 
            msg.linear.x = 0.2
            msg.angular.z = self.angular_vel[action]
>>>>>>> Stashed changes
        self.cmd_vel_pub.publish(msg)

        if self.stop_cmd_vel_timer is None:
            self.prev_goal_distance = self.init_goal_distance
            self.stop_cmd_vel_timer = self.create_timer(0.8, self.timer_callback)
        else:
            self.destroy_timer(self.stop_cmd_vel_timer)
            self.stop_cmd_vel_timer = self.create_timer(0.8, self.timer_callback)

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
