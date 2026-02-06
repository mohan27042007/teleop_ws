# filepath: ~/teleop_ws/src/slam_toolbox_config/slam_toolbox_config/launch/slam.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    pkg_dir = get_package_share_directory('slam_toolbox_config')
    # Points to the config file inside the installed package share
    config_file = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ]
        )
    ])