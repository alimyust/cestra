from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import AppendEnvironmentVariable
import xacro
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    cestra_turtle_dir = get_package_share_directory('cestra_turtle')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_teleop = LaunchConfiguration("use_teleop")
    world_name = LaunchConfiguration('world_name')
    camera_topic = LaunchConfiguration('camera_topic')

    rviz_config_path = PathJoinSubstitution([
        cestra_turtle_dir,
        'config',
        'cestra_turtle.rviz'
        ])

    world = PathJoinSubstitution([
        cestra_turtle_dir,
        'worlds',
        world_name,
    ])

    declare_world = DeclareLaunchArgument(
        'world_name',
        default_value='redlane_v1.world',
        description='World file located in the cestra_turtle/worlds directory'
    )

    declare_teleop = DeclareLaunchArgument(
        'use_teleop',
        default_value='false',
        description='Wheter to use turtlebot3 teleop or not'
    )

    declare_camera_topic = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Simulation camera image topic for line following'
    )

    turtle_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch','spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()

    )

    turtle_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir,'launch','robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

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

    rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path]
    )

    line_follower = Node(
        package='cestra_turtle',
        executable='line_follower_classic',
        name='line_follower_classic',
        output='screen',
        parameters=[{
            'use_webcam': False,
            'camera_topic': camera_topic,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }]
    )
            
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models'))
    
    return LaunchDescription([
        declare_world,
        declare_teleop,
        declare_camera_topic,

        turtle_rsp,
        turtle_spawn,

        gzserver_cmd,
        gzclient_cmd,

        teleop_keyboard,
        rviz2,
        line_follower,
        set_env_vars_resources,
    ])
