from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_nav',
            executable='teleop_keyboard.py',
            name='teleop_keyboard',
            output='screen'
        )
    ])
