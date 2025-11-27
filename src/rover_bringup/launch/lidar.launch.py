from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    lidar_pkg = get_package_share_directory('sllidar_ros2')

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'sllidar_a1_launch.py')
        )
    )

    return LaunchDescription([
        lidar_launch
    ])
