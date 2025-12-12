
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_nav',
            executable='pico_bridge',
            name='pico_bridge',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baudrate': 115200,
                'encoder_ticks_per_rev': 320
            }]
        ),
    ])
