from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_bringup = get_package_share_directory('rover_bringup')

    # -------- Arguments --------
    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz'
    )

    # -------- Include Description --------
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'description.launch.py')
        )
    )

    # -------- Include Lidar --------
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'lidar.launch.py')
        )
    )

    # -------- Camera Launch --------
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'camera.launch.py')
        )
    )

    # -------- Optional RViz --------
    rviz_launch = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        use_rviz,
        description_launch,
        lidar_launch,
        camera_launch,
        rviz_launch,

        Node(
            package='wifi_bridge',
            executable='wifi_bridge_node',
            name='wifi_bridge_node',
            output='screen'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_bringup, 'config', 'ekf.yaml')],
        ),

        Node(
            package='arm_serial_bridge',
            executable='arm_serial_bridge',
            name='arm_serial_bridge',
            output='screen'
        ),

        Node(
            package='imu_bridge',
            executable='imu_bridge',
            name='imu_bridge',
            output='screen'
        ),

    ])
