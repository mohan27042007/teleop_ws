from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_cam',
            namespace='camera',
            parameters=[
                # Correct USB camera
                {'video_device': '/dev/video2'},

                # Lock resolution
                {'image_size': [640, 480]},

                # Use MJPG (hardware compressed)
                {'pixel_format': 'MJPG'},

                # Output for ROS / CV
                {'output_encoding': 'rgb8'},

                # Reduce FPS for stability
                {'frame_rate': 15.0},
            ],
            output='screen'
        )
    ])
