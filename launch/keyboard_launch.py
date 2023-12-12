from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop',
            namespace='base_keyboard_gpt',
            executable='base_keyboard_gpt',
            name='base_keyboard_gpt',
            output='screen',
            parameters=[
                {'speed': 0.5}, {'turn': 1.0}, {'repeat_rate': 0.0}, {'key_timeout': 0.0}
            ]
        ),
    ])