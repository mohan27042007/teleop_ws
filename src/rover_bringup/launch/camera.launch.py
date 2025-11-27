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
                {'video_device': '/dev/video2'},
                {'image_size': [640, 480]},
                {'pixel_format': 'YUYV'},
                {'output_encoding': 'rgb8'},
                {'frame_rate': 30.0},
            ],
            output='screen'
        )
    ])