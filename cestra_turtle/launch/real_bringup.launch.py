



from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    cestra_turtle_dir = get_package_share_directory('cestra_turtle')
    turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')

    use_teleop = LaunchConfiguration('use_teleop')
    use_rviz = LaunchConfiguration('use_rviz')
    use_robot_bringup = LaunchConfiguration('use_robot_bringup')

    rviz_config_path = PathJoinSubstitution([
        cestra_turtle_dir,
        'config',
        'cestra_turtle.rviz'
    ])

    declare_teleop = DeclareLaunchArgument(
        'use_teleop',
        default_value='true',
        description='Launch keyboard teleoperation'
    )

    declare_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2'
    )

    declare_bringup = DeclareLaunchArgument(
        'use_robot_bringup',
        default_value='true',
        description='Launch physical TurtleBot3 hardware drivers'
    )

    # Physical TurtleBot3 bringup
    turtlebot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_bringup_dir,
                'launch',
                'robot.launch.py'
            )
        ),
        condition=IfCondition(use_robot_bringup)
    )

    # Keyboard teleoperation
    teleop_keyboard = ExecuteProcess(
        cmd=[
            'xterm',
            '-e',
            'bash', '-c',
            'source /opt/ros/jazzy/setup.bash && '
            'export TURTLEBOT3_MODEL=burger && '
            'ros2 run turtlebot3_teleop teleop_keyboard; '
            'exec bash'
        ],
        condition=IfCondition(use_teleop),
        output='screen'
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        declare_teleop,
        declare_rviz,
        declare_bringup,

        turtlebot_bringup,
        teleop_keyboard,
        rviz2,
    ])
