

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare("robot_nav").find("robot_nav")

    # --- 1. Archivos de entrada ---
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"])
    map_yaml = PathJoinSubstitution([pkg_share, "map", "three_map.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "nav2_navigation.rviz"])

    # --- 2. Incluir la localización que ya funciona ---
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "localization.launch.py"])
        )
    )

    # --- 3. Nav2 Nodos mínimos ---
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": 'lab_1.yaml'}]
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params]
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params]
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params]
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": [
                "map_server",
                "amcl",
                "planner_server",
                "controller_server",
                "bt_navigator"
            ]
        }]
    )
    
    # --- 4. RViz con herramienta de navegación ---
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    return LaunchDescription([
        localization_launch,
        map_server,
        amcl,
        planner_server,
        controller_server,
        bt_navigator,
        lifecycle_manager,
        rviz2
    ])
