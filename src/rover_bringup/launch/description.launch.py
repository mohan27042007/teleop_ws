from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    description_pkg = get_package_share_directory('rover_description')

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'description.launch.py')
        )
    )

    return LaunchDescription([
        description_launch
    ])
