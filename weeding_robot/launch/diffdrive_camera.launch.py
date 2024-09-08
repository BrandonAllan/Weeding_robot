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
                'pixel_format': 'YUYV',    # Supported pixel format
                'time_per_frame': [1, 30], # 30 FPS
                'camera_frame_id': 'camera_link_optical',
                'brightness': 0,           # Adjust based on valid range
                'contrast': 32,
                'saturation': 64,
                'sharpness': 3,
                'hue': 0,                  # Set hue to a valid value
                'gamma': 100,              # Set gamma to a valid value
                'gain': 0,                 # Set gain to a valid value
                'white_balance_temperature': 4600,  # Set white_balance_temperature to a valid value
                'auto_exposure': 3,
                'white_balance_auto': True,
                'backlight_compensation': 1  # Set backlight_compensation to a valid value
            }],
        )
    ])

if __name__ == '__main__':
    generate_launch_description()