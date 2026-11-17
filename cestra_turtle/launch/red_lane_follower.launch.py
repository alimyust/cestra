
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # =========================================================
    # LAUNCH ARGUMENTS
    # =========================================================

    stages = LaunchConfiguration('stages')
    use_sim = LaunchConfiguration('use_sim')
    camera_id = LaunchConfiguration('camera_id')
    camera_topic = LaunchConfiguration('camera_topic')
    show_images = LaunchConfiguration('show_images')

    declare_stages = DeclareLaunchArgument(
        'stages',
        default_value='6',
        description='Number of vision pipeline stages to launch (1-6)'
    )

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation camera instead of OpenCV webcam'
    )

    declare_camera_id = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='OpenCV webcam device ID'
    )

    declare_camera_topic = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image input topic'
    )

    declare_show_images = DeclareLaunchArgument(
        'show_images',
        default_value='false',
        description='Enable OpenCV visualization windows'
    )

    # =========================================================
    # STAGE CONDITIONS
    # =========================================================

    def stage_condition(stage):
        return IfCondition(
            PythonExpression([
                'int(', stages, ') >= ', str(stage)
            ])
        )

    # =========================================================
    # CAMERA SOURCE
    # =========================================================

    webcam = Node(
        package='cestra_turtle',
        executable='webcam',
        name='webcam_node',
        output='screen',

        parameters=[{
            'camera_id': ParameterValue(
                camera_id,
                value_type=int
            ),
            'width': 640,
            'height': 480,
            'fps': 30.0,
            'show_image': ParameterValue(
                show_images,
                value_type=bool
            ),
        }],

        # Only run webcam when NOT simulating
        condition=UnlessCondition(use_sim)
    )

    # =========================================================
    # STAGE 1: CROP
    # =========================================================

    crop = Node(
        package='cestra_turtle',
        executable='crop',
        name='crop_node',
        output='screen',

        parameters=[{
            'show_image': ParameterValue(
                show_images,
                value_type=bool
            ),
            'crop_start': 0.5,
            'crop_end': 1.0,
        }],

        remappings=[
            ('/camera/image_raw', camera_topic),
        ],

        condition=stage_condition(1)
    )

    # =========================================================
    # STAGE 2: BLUR
    # =========================================================

    blur = Node(
        package='cestra_turtle',
        executable='blur',
        name='blur_node',
        output='screen',

        parameters=[{
            'show_image': ParameterValue(
                show_images,
                value_type=bool
            ),
            'kernel_size': 5,
        }],

        condition=stage_condition(2)
    )

    # =========================================================
    # STAGE 3: RED THRESHOLD
    # =========================================================

    threshold = Node(
        package='cestra_turtle',
        executable='threshold',
        name='threshold_node',
        output='screen',

        parameters=[{
            'show_image': ParameterValue(
                show_images,
                value_type=bool
            ),
            'low_h1': 0,
            'high_h1': 10,
            'low_h2': 170,
            'high_h2': 180,
            'saturation_min': 100,
            'value_min': 50,
        }],

        condition=stage_condition(3)
    )

    # =========================================================
    # STAGE 4: MORPHOLOGICAL CLEANUP
    # =========================================================

    cleanup = Node(
        package='cestra_turtle',
        executable='cleanup',
        name='cleanup_node',
        output='screen',

        parameters=[{
            'show_image': ParameterValue(
                show_images,
                value_type=bool
            ),
            'kernel_size': 5,
            'open_iterations': 1,
            'close_iterations': 2,
        }],

        condition=stage_condition(4)
    )

    # =========================================================
    # STAGE 5: LANE OFFSET
    # =========================================================

    offset = Node(
        package='cestra_turtle',
        executable='offset',
        name='lane_offset_node',
        output='screen',

        parameters=[{
            'show_image': ParameterValue(
                show_images,
                value_type=bool
            ),
            'min_pixels': 100,
        }],

        condition=stage_condition(5)
    )

    # =========================================================
    # STAGE 6: STEERING CONTROLLER
    # =========================================================

    steering = Node(
        package='cestra_turtle',
        executable='steering',
        name='steering_node',
        output='screen',

        parameters=[{
            'kp': 0.005,
            'linear_speed': 0.1,
            'max_angular': 1.5,
            'timeout': 0.5,
        }],

        condition=stage_condition(6)
    )

    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================

    return LaunchDescription([

        declare_stages,
        declare_use_sim,
        declare_camera_id,
        declare_camera_topic,
        declare_show_images,

        webcam,

        crop,
        blur,
        threshold,
        cleanup,
        offset,
        steering,

    ])
