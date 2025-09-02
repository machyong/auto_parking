#!/usr/bin/env python3
import os
import sys
import time
import math
import random
import collections
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty  # 맨 위 import



import tensorflow as tf
from tensorflow.keras.models import Sequential, load_model
from tensorflow.keras.layers import Dense, Input
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.losses import MeanSquaredError

from std_msgs.msg import Bool
from turtlebot3_msgs.srv import Dqn

# GPU 비활성 (테스트 일관성)
tf.config.set_visible_devices([], 'GPU')


class DQNTest(Node):
    def __init__(self):
        super().__init__('dqn_test')

        # ----- 하이퍼파라미터 / 상태 -----
        self.state_size = 2
        self.action_size = 10

        # 테스트 모드: 기본 exploit. (원하면 True로 바꾸고 ε 세팅)
        self.train_mode = False
        self.epsilon_min = 0.05
        self.epsilon = 1.0
        self.epsilon_decay = 2000.0
        self.step_counter = 0

        # 페이징/플래그
        self.training_active = False   # pre-phase(직진) → train-phase 전환 판단
        self.start_triggered = False   # /parking_zone_detected True 수신 시
        self.pre_step = 0              # pre-phase 스텝 카운터

        # 점수 추적
        self.score = 0.0
        self.sum_max_q = 0.0
        self.local_step = 0

        # 상태 버퍼
        self.last_state = None         # (1, state_size)
        self.curr_state = None         # (1, state_size)
        self.init_flag = True          # 서비스 요청의 init

        self.cb_group = ReentrantCallbackGroup()
        self.memory = collections.deque(maxlen=1000000)  # 테스트에선 사용 안 함

        self.make_environment_client  = self.create_client(Empty, 'make_environment')
        self.reset_environment_client = self.create_client(Dqn,   'reset_environment')

        # ----- 모델 -----
        self.model = self.build_model()
        self.model_dir_path = os.path.join('/home/yong/auto_parking', 'saved_model')
        model_path = os.path.join(self.model_dir_path, 'episode3300.h5')

        loaded_model = load_model(
            model_path,
            compile=False,
            custom_objects={'mse': MeanSquaredError()}
        )
        self.model.set_weights(loaded_model.get_weights())

        # 워밍업: 첫 predict JIT
        dummy = np.zeros((1, self.state_size), dtype=np.float32)
        _ = self.model.predict(dummy, verbose=0)

        # ----- 통신 -----
        self.parking_sub = self.create_subscription(
            Bool,
            '/parking_zone_detected',
            self.parking_sub_callback,
            10,
            callback_group=self.cb_group
        )
        self.rl_agent_interface_client = self.create_client(Dqn, 'rl_agent_interface')

        # ----- 타이머로 스텝 수행 -----
        # 0.31초 주기로 environment의 timer와 유사한 cadence 유지
        self.timer = self.create_timer(0.31, self.step_loop, callback_group=self.cb_group)

        self.get_logger().info('dqn_test node ready. Waiting for /parking_zone_detected or pre-step timeout...')
        self.env_make()           # 환경 구성 (선택이지만 있으면 안정적)
        time.sleep(0.5)
        self.reset_environment()  # 첫 에피소드 시작 상태 세팅

    def env_make(self):
        while not self.make_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('make_environment not available, retrying...')
        self.make_environment_client.call_async(Empty.Request())

    def reset_environment(self):
        while not self.reset_environment_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('reset_environment not available, retrying...')
        future = self.reset_environment_client.call_async(Dqn.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('reset_environment failed or timed out')
            return False
        # 초기 상태를 현재 상태로 반영
        st = np.asarray(future.result().state, dtype=np.float32).reshape(1, -1)
        if st.shape[1] != self.state_size:
            self.get_logger().error(f'reset state size {st.shape[1]} != {self.state_size}')
            return False
        self.last_state = None
        self.curr_state = st
        self.init_flag = True
        self.score = 0.0
        self.sum_max_q = 0.0
        self.local_step = 0
        self.pre_step = 0
        self.training_active = False
        self.start_triggered = False
        return True


    # ------------------- 모델 -------------------
    def build_model(self):
        model = Sequential([
            Input(shape=(self.state_size,)),
            Dense(512, activation='relu', kernel_initializer='lecun_uniform'),
            Dense(256, activation='relu', kernel_initializer='lecun_uniform'),
            Dense(128, activation='relu', kernel_initializer='lecun_uniform'),
            Dense(32, activation='relu', kernel_initializer='lecun_uniform'),
            Dense(self.action_size, activation='linear', kernel_initializer='lecun_uniform'),
        ])
        model.compile(loss=MeanSquaredError(), optimizer=Adam(learning_rate=7e-4))
        return model

    # ------------------- 정책 -------------------
    def get_action(self, state_1xN: np.ndarray) -> int:
        """
        state_1xN: shape (1, state_size)
        """
        if self.train_mode:
            self.step_counter += 1
            self.epsilon = self.epsilon_min + (1.0 - self.epsilon_min) * math.exp(
                - self.step_counter / self.epsilon_decay
            )
            if random.random() < self.epsilon:
                return random.randint(0, self.action_size - 1)

        q_values = self.model.predict(state_1xN, verbose=0)
        return int(np.argmax(q_values, axis=1)[0])

    # ------------------- 콜백 -------------------
    def parking_sub_callback(self, msg: Bool):
        if msg.data:
            self.start_triggered = True
            self.get_logger().info('Parking zone detected → training phase will start.')

    # ------------------- 메인 루프 -------------------
    def step_loop(self):
        # 서비스 대기 (처음 몇 틱)
        if not self.rl_agent_interface_client.wait_for_service(timeout_sec=0.0):
            # 서비스 대기 중에도 스핀은 타이머가 돌고 있으니 OK
            return

        # pre-phase → train-phase 전환
        if (not self.training_active) and (self.start_triggered or self.pre_step > 600):
            self.training_active = True
            self.start_triggered = False
            self.local_step = 0
            self.score = 0.0
            self.sum_max_q = 0.0
            self.get_logger().info('Entering training phase.')

        # 액션 결정
        if not self.training_active:
            action = 2  # 직진
            self.pre_step += 1
        else:
            # 상태가 없으면 더미로 시작 (첫 틱 보호)
            if self.curr_state is None:
                state_1xN = np.zeros((1, self.state_size), dtype=np.float32)
            else:
                state_1xN = self.curr_state

            # Q 통계
            q_values = self.model.predict(state_1xN, verbose=0)
            self.sum_max_q += float(np.max(q_values))

            action = self.get_action(state_1xN)
            self.local_step += 1

        # 서비스 요청 작성
        req = Dqn.Request()
        req.action = int(action)
        req.init = bool(self.init_flag)

        # 비동기 호출 + 완료까지 스핀
        future = self.rl_agent_interface_client.call_async(req)
        rclpy.task.Future  # type hint용 no-op
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if not future.done() or future.result() is None:
            self.get_logger().warn('Service call timed out or failed; skipping this tick.')
            return

        # 응답 처리
        res = future.result()
        # state reshape to (1, state_size)
        st = np.asarray(res.state, dtype=np.float32).reshape(1, -1)
        if st.shape[1] != self.state_size:
            self.get_logger().error(f'Invalid state size {st.shape[1]} (expected {self.state_size}).')
            return

        self.last_state = self.curr_state
        self.curr_state = st

        self.score += float(res.reward)
        done = bool(res.done)

        # init 플래그는 첫 호출 후 False
        if self.init_flag:
            self.init_flag = False

        if done:
            # 에피소드 종료 로그
            avg_max_q = (self.sum_max_q / max(1, self.local_step)) if self.training_active else 0.0
            self.get_logger().info(
                f'[TEST] done. score={self.score:.3f}, steps={self.local_step}, avg_max_q={avg_max_q:.4f}'
            )
            # 다음 에피소드 준비
            self.training_active = False
            while self.start_triggered == True:
                self.pre_step = 0
                self.local_step = 0
                self.score = 0.0
                self.sum_max_q = 0.0
                self.init_flag = True
                self.last_state = None
                self.curr_state = None
                self.start_triggered = False
            ok = self.reset_environment()
            if ok:
                self.get_logger().info('Environment reset. Waiting for parking detect or pre-step timeout...')
            else:
                self.get_logger().warn('Environment reset failed; will continue anyway.')
            return  # 이 틱 종료


def main(argv=None):
    rclpy.init(args=argv if argv else sys.argv)
    node = DQNTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
