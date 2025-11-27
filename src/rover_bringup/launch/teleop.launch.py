from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_key',
        prefix='xterm -e',   # opens keyboard teleop in a terminal window
        output='screen'
    )

    return LaunchDescription([
        teleop
    ])
