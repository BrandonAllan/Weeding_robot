from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():



    return LaunchDescription([

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            namespace='camera',
            parameters=[{
                'video_device': '/dev/video2',
                'image_size': [640,640],
                'time_per_frame': [1, 30],
                'camera_frame_id': 'camera_link_optical'
                }]
    )
    ])