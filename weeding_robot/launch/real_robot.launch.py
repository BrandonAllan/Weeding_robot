import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    description_package = 'agri_robot_description'
    package_name = 'weeding_robot'
    joystick_package = 'robot_joystick'
    world_file = "./src/agri_robot_description/worlds/crop_field.world"


    gazebo_params = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo.yaml')

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(description_package), 'launch', 'display.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(joystick_package),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file, 'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params}.items()
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'agri_robot',
                                   "-x", '-5.0',
                                   "-y", '4.85',
                                   "-z", '1.1'],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output='screen'
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output='screen'
    )

    # Combine multiple actions into a single timer action
    combined_actions = [
        spawn_entity, 
        diff_drive_spawner,  
        joint_broad_spawner  
    ]

    # Add a timer action to delay before executing the combined actions
    delay_before_actions = TimerAction(
        period=5.0,  # Adjust the delay duration as needed
        actions=combined_actions  # Execute the combined actions after the delay
    )

    return LaunchDescription([
        display,
        #joystick,
        twist_mux,
        gazebo,
        delay_before_actions,
    ])
