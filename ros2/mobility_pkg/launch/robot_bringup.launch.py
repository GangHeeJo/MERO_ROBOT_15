import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 모빌리티(UGV) 하위 제어 노드
        Node(
            package='mobility_pkg',
            executable='ugv_controller_node',
            name='ugv_node',
            output='screen',
            emulate_tty=True  # 터미널 출력에 색상을 입혀 에러/경고 식별을 쉽게 함
        ),
        # 2. 카메라 영상 취득 노드
        Node(
            package='mobility_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen',
            emulate_tty=True
        ),
        # 3. YOLO 비전 인식 노드
        Node(
            package='mobility_pkg',
            executable='yolo_vision_node',
            name='yolo_vision_node',
            output='screen',
            emulate_tty=True
        ),
        # 4. 메인 대뇌 (제어 판단) 노드
        Node(
            package='mobility_pkg',
            executable='main_decision_node',
            name='main_decision_node',
            output='screen',
            emulate_tty=True
        ),
        # 5. 그리퍼 제어 노드
        Node(
            package='mobility_pkg',
            executable='gripper_node',
            name='gripper_node',
            output='screen',
            emulate_tty=True
        )
    ])