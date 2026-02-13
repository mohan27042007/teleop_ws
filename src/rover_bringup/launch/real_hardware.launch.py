import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_rover_bringup = get_package_share_directory('rover_bringup')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    pkg_slam_toolbox_config = get_package_share_directory('slam_toolbox_config')
    slam_params = os.path.join(pkg_slam_toolbox_config, 'config', 'mapper_params_online_async.yaml')

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

    # ---------------------------------------------------------
    # CONDITIONAL LAUNCH LOGIC (Bypassing nav2_bringup crash)
    # ---------------------------------------------------------

    # A. SLAM Toolbox (Only if slam=True)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'params_file': slam_params, 'use_sim_time': 'False'}.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # B. Localization (AMCL/Map Server) (Only if slam=False)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params,
            'use_sim_time': 'False',
            'autostart': 'True'
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('slam'))
    )

    # C. Navigation2 (Core Stack - Always Launched)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'False',
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([
        slam_arg,
        
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

        # 3. Wifi Bridge
        Node(
            package='wifi_bridge',
            executable='wifi_bridge_node',
            name='wifi_bridge',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel_safe')]
        ),

        # 4. Twist Mux
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[twist_mux_config],
            remappings=[('/cmd_vel_out', '/cmd_vel_raw')]
        ),

        # 5. Perception Guard
        Node(
            package='perception_guard',
            executable='perception_guard_node',
            name='perception_guard',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel_safe')]
        ),

        # 6. EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ), 

        # 7. Navigation Stack (Split)
        slam_launch,
        localization_launch,
        navigation_launch
    ])