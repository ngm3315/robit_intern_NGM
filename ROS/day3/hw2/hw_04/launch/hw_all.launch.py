from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # yaml 경로
    config = os.path.join(
        get_package_share_directory('hw_04'),
        'config',
        'arm_params.yaml'
    )

    return LaunchDescription([
        # turtlesim 과제 노드
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='hw_01',
            executable='hw_01_node',
            name='drawer',
            output='screen'
        ),

        # 로봇팔 과제 노드
        Node(
            package='hw_04',
            executable='hw_04_node',
            name='robot_arm',
            parameters=[config],
            output='screen'
        )
    ])
