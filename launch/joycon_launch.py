from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop',
            namespace='base_joycon',
            executable='base_joycon',
            name='base_joycon',
            output='screen',
            parameters=[
                {'l_scale': 0.6}, {'a_scale': 0.8},
            ]
        ),
    ])
