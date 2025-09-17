from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    scale_lin = LaunchConfiguration('scale_linear', default='1.0')
    scale_ang = LaunchConfiguration('scale_angular', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument('scale_linear', default_value='1.0'),
        DeclareLaunchArgument('scale_angular', default_value='1.0'),

        Node(  # DS4 드라이버 노드
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'autorepeat_rate': 50.0}]
        ),
        Node(  # DS4→Twist
            package='dual4_publisher',
            executable='publisher_node',
            name='ds4_to_twist',
            output='screen',
            parameters=[{
                'axis_linear': 1,
                'axis_angular': 3,
                'scale_linear': scale_lin,
                'scale_angular': scale_ang,
                'deadzone': 0.05
            }]
        ),
    ])
