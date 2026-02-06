import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_rover_bringup = get_package_share_directory('rover_bringup')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # Config files
    twist_mux_config = os.path.join(pkg_rover_bringup, 'config', 'twist_mux.yaml')
    nav2_params = os.path.join(pkg_rover_bringup, 'config', 'nav2_params_real.yaml')
    map_file = os.path.join(pkg_rover_bringup, 'maps', 'blank_map.yaml')
    ekf_config = os.path.join(pkg_rover_bringup, 'config', 'ekf.yaml')

    # -------- Arguments --------
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to run SLAM (True) or Localization (False)'
    )

    return LaunchDescription([
        slam_arg,
        
        # 1. Start Robot Description (URDF + TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_rover_bringup, 'launch', 'description.launch.py')
            )
        ),

        # 2. Start RPLidar (Fixed Frame ID)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sllidar, 'launch', 'sllidar_a1_launch.py')
            ),
            launch_arguments={
                'serial_port': '/dev/lidar',  # Updated to use persistent symlink
                'frame_id': 'lidar_link'  # <--- CRITICAL FIX: Match URDF
            }.items()
        ),

        # 3. Start Wifi Bridge (Talks to ESP32)
        Node(
            package='wifi_bridge',
            executable='wifi_bridge_node',
            name='wifi_bridge',
            output='screen',
            # Wifi bridge subscribes to /cmd_vel. 
            # We remap it to /cmd_vel_safe coming from Perception Guard.
            remappings=[('/cmd_vel', '/cmd_vel_safe')]
        ),

        # 4. Twist Mux (Takes Nav + Teleop -> Outputs to Raw)
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[twist_mux_config],
            remappings=[('/cmd_vel_out', '/cmd_vel_raw')]
        ),

        # 5. Perception Guard (Takes Raw -> Outputs to Safe)
        Node(
            package='perception_guard',
            executable='perception_guard_node',
            name='perception_guard',
            output='screen',
            # Guard listens to /cmd_vel_raw and publishes /cmd_vel.
            # We remap its output to /cmd_vel_safe to go to the bridge
            remappings=[('/cmd_vel', '/cmd_vel_safe')]
        ),

        # 6. EKF (Robot Localization)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ), 

        # 7. Navigation2 (The Brain)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'params_file': nav2_params,
                'use_sim_time': 'False',
                'autostart': 'True',
                'slam': LaunchConfiguration('slam')
            }.items()
        ),
    ])