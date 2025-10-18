import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

from launch_ros.actions import Node
import xacro




def generate_launch_description():
    

    bringup_pkg = get_package_share_directory('cestra_bringup')
    description_pkg = get_package_share_directory('cestra_description')
    # simulation_pkg = get_package_share_directory('cestra_simulation')
    
    default_world = os.path.join(description_pkg,'worlds','empty.world')    
    robot_description = os.path.join(description_pkg, 'urdf', 'cestra_swerve.urdf')
    robot_description_config = xacro.process_file(robot_description).toxml()


    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    swerve_sim = Node(
        package='cestra_drive',
        executable='swerve_sim',
        name='swerve_sim',
        output='screen'
        )


    world = LaunchConfiguration('world')
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
        
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Run the spawner node from the ros_gz_sim package.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'cestra_swerve','-z', '1.0'],
                        output='screen'
    )

    bridge_params = os.path.join(bringup_pkg,'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )



    # Launch them all!
    return LaunchDescription([
        rsp,
        
        world_arg,
        gazebo,
        rviz_launch,

        spawn_entity,
        swerve_sim,
        ros_gz_bridge,

    ])