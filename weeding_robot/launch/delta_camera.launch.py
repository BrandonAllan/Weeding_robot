import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video2',
                'image_size': [800, 600],  # Supported resolution
            }],
        )
    ])

if __name__ == '__main__':
    generate_launch_description()