from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1) 기존 런치 포함
    other_pkg = 'turtlebot3_gazebo'  # ← 바꾸기
    other_launch_path = os.path.join(
        get_package_share_directory(other_pkg),
        'launch',
        'turtlebot3_autorace_2020.launch.py'   # ← 기존 런치 파일명
    )
    include_other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_path)
    )

    # 2) 원래 ros2 run 하던 노드 3개 등록
    gazebo_set= Node(
        package='turtlebot3_dqn',      # ← 패키지명
        executable='dqn_gazebo',   # ← 실행파일명
        name='gazebot_set',
        output='screen'
    )

    rear_camera_detector = Node(
        package='turtlebot3_dqn',
        executable='parkingline_detect',
        name='rear_camera_detector',
        output='screen'
    )

    right_camera_detector = Node(
        package='turtlebot3_dqn',
        executable='parking_detector',
        name='right_camera_detector',
        output='screen'
    )

    # 3) LaunchDescription에 전부 추가
    return LaunchDescription([
        include_other_launch,
        gazebo_set,
        rear_camera_detector,
        right_camera_detector
    ])
