import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_rover_bringup = get_package_share_directory('rover_bringup')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')

    # Config files
    twist_mux_config = os.path.join(pkg_rover_bringup, 'config', 'twist_mux.yaml')
    ekf_config = os.path.join(pkg_rover_bringup, 'config', 'ekf.yaml')

    return LaunchDescription([
        
        # 1. Start Robot Description (URDF + TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_rover_bringup, 'launch', 'description.launch.py')
            )
        ),

        # 2. Start RPLidar
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sllidar, 'launch', 'sllidar_a1_launch.py')
            ),
            launch_arguments=[
                ('serial_port', '/dev/lidar'),
                ('frame_id', 'lidar_link')
            ]
        ),

        # 3. Wifi Bridge (Odom Source)
        Node(
            package='wifi_bridge',
            executable='wifi_bridge_node',
            name='wifi_bridge',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel_safe')]
        ),

        # 4. Twist Mux (Safety)
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[twist_mux_config],
            remappings=[('/cmd_vel_out', '/cmd_vel_raw')]
        ),

        # 5. Perception Guard (Safety)
        Node(
            package='perception_guard',
            executable='perception_guard_node',
            name='perception_guard',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel_safe')]
        ),

        # 6. EKF (TF Source)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ), 
    ])
