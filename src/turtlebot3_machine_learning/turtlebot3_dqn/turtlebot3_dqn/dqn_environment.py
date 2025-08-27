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
from datetime import datetime

from turtlebot3_msgs.srv import Dqn
from turtlebot3_msgs.srv import Goal
from tqdm import tqdm
# from std_msgs.msg import Float64
# from gazebo_msgs.msg import ContactsState  # 맨 위 import 섹션에 추가


from std_msgs.msg import Float64, Bool

ROS_DISTRO = os.environ.get('ROS_DISTRO')


class RLEnvironment(Node):

    def __init__(self):
        super().__init__('rl_environment')
        self.goal_pose_x = 0.2648
        self.goal_pose_y = 1.1942
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.episode_count = 1
        self.action_size = 10
        self.max_step = 300

        self.done = False
        self.fail = False
        self.succeed = False
        self.parkingline_ratio = 0.0
        self.parking_detect = False
        self.collision_flag = False
        self.goal_angle = 0.0
        self.goal_distance = 1.0
        self.init_goal_distance = 0.7
        self.scan_ranges = []
        self.front_ranges = []
        self.min_obstacle_distance = 10.0
        self.is_front_min_actual_front = False
        self.pbar = tqdm(total=self.max_step, desc="Episode Progress", position=0, leave=True)
        self.model_dir_path = os.path.join(
            '/home/dykim/auto_parking',
            'saved_model'
        )

        self.local_step = 0
        self.stop_cmd_vel_timer = None
        self.angular_vel = [
            [ 0.1,  0.6], [ 0.1,  0.3], [ 0.1,  0.0], [ 0.1, -0.3], [ 0.1, -0.6],
            [-0.1,  0.6], [-0.1,  0.3], [-0.1,  0.0], [-0.1, -0.3], [-0.1, -0.6]
        ]
        qos = QoSProfile(depth=10)

        if ROS_DISTRO == 'humble':
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_sub_callback,
            qos
        )
        self.rear_yellow_ratio = self.create_subscription(
            Float64,
            '/rear/ratio',
            self.ratio_callback,
            10
        )

        self.collision_flag_sub = self.create_subscription(
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

        self.parking_sub = self.create_subscription(
            Bool,
            '/parking_zone_detected',
            self.parking_sub_callback,
            10,
            callback_group=self.clients_callback_group
        )
        # ===== [CHANGE] 후면주차 방향 =====
        self.goal_yaw = 0.0   # 후면주차: 차량 헤딩이 슬롯 밖(0) 향하도록

        # ===== [NEW] 보상 예산(점수 분배) =====
        self.OUTCOME_BUDGET = 25.0   # 성공/실패 터미널 보상 절대값
        self.STEP_BUDGET    = 25.0   # 스텝 패널티 총합
        # 텔레스코핑(PBRS) 총합 상한 25.0

        # ===== [NEW] 잠재함수(PBRS) 계수 및 감쇠 =====
        self.GAMMA = 0.99
        self.SHAPING_LAMBDA = 0.7     # 텔레스코핑 세기(스텝 shaping 강도)
        self.KD, self.KY, self.KX, self.KROI = 1.0, 0.8, 0.8, 0.5
        #  d:거리, y:Yaw오차, x:깊이오차, roi:후방선 신뢰

        # ===== [NEW] 캡처존/목표 정렬 기준 =====
        self.TARGET_Y = 1.1905         # 슬롯 중앙 Y
        self.TARGET_X_BACK = 0.2525      # 원하는 최종 X(더 작을수록 더 깊이 들어간 상태면 조정)
        self.CAPTURE_Y_MIN, self.CAPTURE_Y_MAX = 1.11, 1.27
        self.CAPTURE_X_MAX = 0.33

        # ===== [NEW] 상황 힌트(소규모 가중) =====
        self.BACK_IN_CAPTURE_BONUS = 1.0   # 캡처존에서 후진 보너스(+)
        self.FWD_IN_CAPTURE_PENALTY = 1.0  # 캡처존에서 전진 감점(−)
        self.SMOOTH_PENALTY = 0.05         # 급변 억제

        # ===== [NEW] 텔레스코핑/스무딩 저장 변수 =====
        self._phi_prev = None
        self._prev_action = None
        self._d_norm_prev = 0.0
        self._yaw_align_prev = 0.0


        #          # 최종 정렬하고 싶은 방향 (예: 0 rad)
        self.last_action = None       # 직전 행동 기억(보상에서 전/후진 판단용)

    # ===== [NEW] 보조 함수들 =====
    def _yaw_err(self):
        return abs(self._wrap(self.goal_yaw - self.robot_pose_theta))

    def _y_err(self):
        return abs(self.robot_pose_y - self.TARGET_Y)

    def _depth_err(self):
        # 목표보다 덜 들어갔으면 양수(벌금), 더 들어갔으면 0
        return max(0.0, self.robot_pose_x - self.TARGET_X_BACK)

    def _roi_norm(self):
        r = float(self.parkingline_ratio)
        if r <= 0.6:
            r *= 0.5
        return max(0.0, min(1.0, r))  # 0~1 정규화

    def _phi(self):
        # 잠재함수: 값이 작아질수록 좋은 상태
        return ( self.KD   * float(self.goal_distance)
            + self.KY   * self._yaw_err()
            + self.KX   * self._depth_err()
            + self.KROI * (1.0 - self._roi_norm()) )

    def _in_capture_zone(self):
        return (self.robot_pose_x <= self.CAPTURE_X_MAX) and \
            (self.CAPTURE_Y_MIN <= self.robot_pose_y <= self.CAPTURE_Y_MAX)

    def _capture_zone_hint(self, action_idx):
        if action_idx is None:
            return 0.0
        lin, ang = self.angular_vel[action_idx]
        if self._in_capture_zone():
            if lin < 0:
                return +self.BACK_IN_CAPTURE_BONUS
            elif lin > 0:
                return -self.FWD_IN_CAPTURE_PENALTY
        return 0.0

    def _smooth_penalty(self, prev_action, cur_action):
        if prev_action is None or cur_action is None:
            return 0.0
        lin_p, ang_p = self.angular_vel[prev_action]
        lin_c, ang_c = self.angular_vel[cur_action]
        return -self.SMOOTH_PENALTY * (abs(lin_c - lin_p) + abs(ang_c - ang_p))

    def _norm_distance(self):
        init_d = max(self.init_goal_distance, 1e-6)
        d_ratio = min(1.0, float(self.goal_distance / init_d))
        return 1.0 - d_ratio  # 0(멀다) → 1(가깝다)

    def _norm_yaw_align(self):
        return 1.0 - (self._yaw_err() / math.pi)  # 0(정반대) → 1(정렬)

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
        # 초기 흔들림 방지용 정지 명령
        self.settle_robot(duration=0.5, hz=20)

        state = self.calculate_state()
        self.init_goal_distance = state[0]
        response.state = state
        
        self._phi_prev = None
        self._prev_action = None
        self._d_norm_prev = 0.0
        self._yaw_align_prev = 0.0
        self.last_action = None
        self.local_step = 0
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

    # 차량 각도 차이 계산용 함수
    def _wrap(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a
    
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
        

    def calculate_state(self):
        state = []
        yaw_error = self._wrap(self.goal_yaw - self.robot_pose_theta)
        state.append(float(self.goal_distance))
        state.append(float(yaw_error))
        self.local_step += 1
        self.get_logger().info(f'current step : {self.local_step}')

        msg = Twist() 
        msg.linear.x = 0.
        msg.angular.z = 0.
        if (0.966 <= self.robot_pose_y <= 1.413) and self.robot_pose_x <= 0.25:
            self.get_logger().info('Goal Reached')
            self.succeed = True
            self.done = True

            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(msg)
                time.sleep(1.)
            self.local_step = 0
            self.episode_count += 1
            self.parking_detect = False
            self.call_task_succeed()

        if self.collision_flag == True:
            self.get_logger().info('Collision happened')
            self.fail = True
            self.done = True
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(msg)
                time.sleep(1.)
            self.parking_detect = False
            self.local_step = 0
            self.episode_count += 1
            self.call_task_failed()

        # 시간초과
        if self.local_step == self.max_step:
            self.get_logger().info('Time out!')
            self.fail = True
            self.done = True
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(msg)
                time.sleep(1.)
            self.parking_detect = False
            self.episode_count += 1
            self.local_step = 0
            self.call_task_failed()

        if self.robot_pose_y <= 0.74:
            self.get_logger().info('Time out!1')
            self.fail = True
            self.done = True
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(msg)
                time.sleep(1.)
            self.parking_detect = False
            self.episode_count += 1
            self.local_step = 0
            self.call_task_failed()

        if self.robot_pose_y >= 1.86 and self.parking_detect == True:
            self.get_logger().info('Time out!2')
            self.fail = True
            self.done = True
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(msg)
                time.sleep(1.)
            self.parking_detect = False
            self.episode_count += 1
            self.local_step = 0
            self.call_task_failed()

        return state
    
    def parking_sub_callback(self, msg):
        self.parking_detect = msg.data

    def calculate_reward(self):
        # === 스텝 페널티(총 −STEP_BUDGET가 되도록 균등 분배) ===
        step_pen = - self.STEP_BUDGET / float(self.max_step)

        # === 텔레스코핑(PBRS) shaping: λ[γΦ(s′) − Φ(s)] ===
        phi_now = self._phi()
        if self._phi_prev is None:
            self._phi_prev = phi_now
        pbrs = self.SHAPING_LAMBDA * (self._phi_prev - self.GAMMA * phi_now)
        self._phi_prev = phi_now

        # === 품질 차분(정규화 지표의 변화량) – 선택/설명용 ===
        d_norm = self._norm_distance()  
        yaw_align = self._norm_yaw_align()
        delta_d = d_norm - self._d_norm_prev
        delta_yaw = yaw_align - self._yaw_align_prev
        # (원하면 여기 delta들을 로그에 남겨 품질 개선량을 모니터링)

        # === 상황 힌트: 캡처존 후진/전진 & 스무딩(소규모) ===
        hint = self._capture_zone_hint(self.last_action)
        smooth = self._smooth_penalty(self._prev_action, self.last_action)

        # === 비-터미널 기본 스텝 보상 ===
        reward = step_pen + pbrs + hint + smooth

        # === 터미널 보상(Outcome) + 절대 품질 보정 ===
        if self.succeed:
            # 절대 품질: Y 정렬, yaw 정렬, 깊이, ROI
            y_align_bonus   = 12.0 * math.exp(-6.0 * self._y_err())
            yaw_align_bonus = 10.0 * math.exp(-2.5 * self._yaw_err())
            depth_bonus     = 10.0 * math.exp(-8.0  * self._depth_err())
            roi_bonus       =  5.0 * self._roi_norm()

            reward += ( + self.OUTCOME_BUDGET
                        + y_align_bonus + yaw_align_bonus + depth_bonus + roi_bonus )

            # 로깅
            today_str = datetime.now().strftime("%m%d")
            result_file = os.path.join(self.model_dir_path, "step_reward" + today_str + ".csv")
            with open(result_file, "a") as f:
                f.write(f"{self.episode_count},{self.local_step},{reward},succeed\n")

        elif self.fail:
            # 실패 절대 품질 감점 + Outcome
            y_bad   = - 8.0 * self._y_err()
            yaw_bad = - 8.0 * self._yaw_err()
            depth_bad = - 8.0 * self._depth_err()
            outcome = "failed"

            if getattr(self, 'collision_flag', False) is True:
                reward += (- self.OUTCOME_BUDGET - 25.0)  # 충돌은 추가 패널티
                outcome = "crash"
            else:
                reward += (- self.OUTCOME_BUDGET)

            reward += (y_bad + yaw_bad + depth_bad)

            today_str = datetime.now().strftime("%m%d")
            result_file = os.path.join(self.model_dir_path, "step_reward" + today_str + ".csv")
            with open(result_file, "a") as f:
                f.write(f"{self.episode_count},{self.local_step},{reward},{outcome}\n")

        # === 다음 스텝 대비 업데이트 ===
        self._d_norm_prev = d_norm
        self._yaw_align_prev = yaw_align
        self._prev_action = self.last_action

        return float(reward)


    # 로봇 동작 수행
    def rl_agent_interface_callback(self, request, response):
        action = int(request.action)
        self.last_action = action
        if ROS_DISTRO == 'humble':
            msg = Twist() 
            msg.linear.x = self.angular_vel[action][0]
            msg.angular.z = self.angular_vel[action][1]
        self.cmd_vel_pub.publish(msg)

        if self.stop_cmd_vel_timer is None:
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
    # 로봇 안정화용
    def settle_robot(self, duration=0.4, hz=20):
        # duration 동안 0속도 명령을 지속해서 퍼블리시
        dt = 1.0/float(hz)
        if ROS_DISTRO == 'humble':
            msg = Twist()
        else:
            msg = TwistStamped()
        end = time.time() + duration
        while time.time() < end:
            if ROS_DISTRO == 'humble':
                self.cmd_vel_pub.publish(Twist())
            else:
                self.cmd_vel_pub.publish(TwistStamped())
            time.sleep(dt)


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
