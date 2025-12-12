from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
import os


def generate_launch_description():

    pkg_share = FindPackageShare('robot_nav').find('robot_nav')

    # Ruta al URDF (aseg�rate de que este archivo exista)
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])

    return LaunchDescription([

        # --- 1. Nodo pico_bridge (control + odometr�a + joint_states)
        Node(
            package='robot_nav',
            executable='pico_bridge',
            name='pico_bridge',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baudrate': 115200,
                'encoder_ticks_per_rev': 10380, # 2325
                'wheel_radius': 0.0325,
                'wheel_separation': 0.239
            }]
        ),

        # --- 2. Publicar URDF (robot_state_publisher)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
              'robot_description': ParameterValue(
                  Command(['xacro ', 
                      PathJoinSubstitution([
                      FindPackageShare('robot_nav'),
                      'urdf',
                      'robot.urdf.xacro'
                ])
            ]),
            value_type=str
            )

            }]
        ),

        # --- 3. Publicar TF estaticas del LIDAR
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    name='lidar_tf',
        #    arguments=['0.0', '0.0', '0.10', '0', '0', '0', 'base_link', 'lidar_link']
        #),

        # --- 4. Nodo del RPLIDAR (ej: rplidar_ros)
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link',
                'angle_compensate': True,
                'scan_mode': 'Standard',
                
            }],
            output='screen',
        ),
        
        
        # --- 5. SLAM Toolbox (modo offline)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
              {"use_sim_time": False},
              PathJoinSubstitution([
                  pkg_share,
                  "config",
                  "localization_params.yaml"
              ]),
              {"mode": "localization"},
              {"do_loop_closing": False},
              {"map_start_at_dock": True},
              {"use_map_saver": False},
              {"map_update_intervsudoal": 9999.0},
            ],
            output='screen'
            
        ),
        
        # --- 5. RViz preconfigurado
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                pkg_share, 'config', 'slam_localization.rviz'
            ])]
        ),

    ])
