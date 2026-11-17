
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # --------------------------------------------------
    # 1. Include existing Gazebo simulation bringup
    # --------------------------------------------------

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cestra_turtle'),
                'launch',
                'turtle_bringup.launch.py'
            ])
        ),
        launch_arguments={
            'world_name': 'redlane_v1.world',
            'use_teleop': 'false',
            'use_sim_time': 'true'
        }.items()
    )

    # --------------------------------------------------
    # 2. Start OpenCV line-following node
    # --------------------------------------------------

    line_follower = Node(
        package='cestra_turtle',
        executable='line_follower',
        name='line_follower',
        output='screen'
    )

    # --------------------------------------------------
    # 3. Return launch description
    # --------------------------------------------------

    return LaunchDescription([
        simulation,
        line_follower
    ])
