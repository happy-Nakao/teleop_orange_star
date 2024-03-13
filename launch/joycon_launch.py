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
        ),
        Node(
            package='joy',
            namespace='joy_node',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
    ])
