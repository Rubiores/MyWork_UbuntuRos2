from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([
        
        # Nodo del RPLIDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 usa 115200
                'frame_id': 'lidar_link',
                'angle_compensate': True
            }],
            output='screen'
        ),

    ])
