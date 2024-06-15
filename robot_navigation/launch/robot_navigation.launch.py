from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    crop_detector = Node(
        package='robot_navigation',
        executable='crop_detector_yolov8',
        remappings=[('/image_in', '/rgb_camera/image_raw')]
    )
    crop_follower = Node(
        package='robot_navigation',
        executable='crop_follower'
    )

    delayed_crop_follower = TimerAction(period=6.0, actions=[crop_follower])

    crop_turning = Node(
        package='robot_navigation',
        executable='crop_turning'
    )

    delayed_crop_turning = RegisterEventHandler(
        event_handler= OnProcessStart(
            target_action=crop_follower,
            on_start=[crop_turning]
        )
    )
    

    return LaunchDescription([
        crop_detector,
        delayed_crop_follower,
        delayed_crop_turning
    ])
