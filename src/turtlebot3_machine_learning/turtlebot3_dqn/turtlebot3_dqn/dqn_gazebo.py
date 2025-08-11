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
# # Authors: Ryan Shim, Gilbert, ChanHyeong Lee

import os
import sys
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty

from turtlebot3_msgs.srv import Goal


ROS_DISTRO = os.environ.get('ROS_DISTRO')

class GazeboInterface(Node):

    def __init__(self):
        super().__init__('gazebo_interface')
        
        self.entity_pose_x = 0.5
        self.entity_pose_y = 1.844

        if ROS_DISTRO == 'humble':
            self.reset_simulation_client = self.create_client(Empty, 'reset_simulation')

        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.initialize_env_service = self.create_service(
            Goal,
            'initialize_env',
            self.initialize_env_callback,
            callback_group=self.callback_group
        )
        self.task_succeed_service = self.create_service(
            Goal,
            'task_succeed',
            self.task_succeed_callback,
            callback_group=self.callback_group
        )
        self.task_failed_service = self.create_service(
            Goal,
            'task_failed',
            self.task_failed_callback,
            callback_group=self.callback_group
        )

    def reset_simulation(self):
        reset_req = Empty.Request()

        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for reset_simulation is not available, waiting ...')

        self.reset_simulation_client.call_async(reset_req)

    def task_succeed_callback(self, request, response):
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        return response
    # 실패시 새로운 목표 좌표 지정
    def task_failed_callback(self, request, response):
        if ROS_DISTRO == 'humble':
            self.reset_simulation()
        time.sleep(0.2)
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        return response

    def initialize_env_callback(self, request, response):
        if ROS_DISTRO == 'humble':
            self.reset_simulation()
        time.sleep(0.2)
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=sys.argv)
    stage_num = sys.argv[1] if len(sys.argv) > 1 else '1'
    gazebo_interface = GazeboInterface(stage_num)
    try:
        while rclpy.ok():
            rclpy.spin_once(gazebo_interface, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        gazebo_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()