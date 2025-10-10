from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    bringup_dir = get_package_share_directory('cestra_bringup')
    drive_dir = get_package_share_directory('cestra_drive')
    description_dir = get_package_share_directory('cestra_description')

    robot_description = os.path.join(description_dir, 'urdf', 'cestra_diff.urdf')
    robot_description_config = xacro.process_file(robot_description).toxml()

    rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        )
    
    test_serial_in = Node(
            package='cestra_drive',
            executable='diff_drive_bridge',
            name='serial_in',
            output='screen',
            parameters=[{'port': '/dev/ttyACM0', 'baudrate': 9600}]
        )
    
    return LaunchDescription([
        rsp,
        test_serial_in
    ])