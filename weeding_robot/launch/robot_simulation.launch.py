import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    description_package = 'agri_robot_description'
    package_name = 'weeding_robot'
    world_file = "./src/agri_robot_description/worlds/crop_field.world"

    gazebo_params = os.path.join(get_package_share_directory(package_name),'config','gazebo.yaml')

    display = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(description_package),'launch','display.launch.py')]),
                    launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file,'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params}.items()
             )
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'agri_robot',
                                   "-x", '-4.0',
                                   "-y", '4.7',
                                   "-z", '1.2'],
                        output='screen')

    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Add a timer action to delay before spawning the entity
    delay_before_spawn = TimerAction(
        period=8.0,  # Adjust the delay duration as needed
        actions=[spawn_entity]  # Spawn the entity after the delay
    )
    
    return LaunchDescription([
        display,    
        gazebo,
        delay_before_spawn,  # Add the delay action before spawning the entity
        diff_drive_spawner,
        joint_broad_spawner
    ])
