import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
 
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    camera_params_file = os.path.join(
        get_package_share_directory('galaxy_camera'), 'config', 'camera_params.yaml')

    params_file = os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'config', 'params.yaml')

    rviz_config_dir = os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'rviz', 'default.rviz')

    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_description'),
        'urdf', 'pioneer_gimbal.urdf.xacro')])

    camera_node = Node(
        package='galaxy_camera',
        executable='camera_node',
        output='screen',
        emulate_tty=True,
        parameters=[camera_params_file],
    )

    detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        emulate_tty=True,
        output='screen',
        parameters=[params_file],
        # arguments=['--ros-args', '--log-level', 'armor_detector:=INFO'],
    )

    processor_node = Node(
        package='armor_processor',
        executable='armor_processor_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
        # arguments=['--ros-args', '--log-level', 'armor_processor:=INFO'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 1000.0}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'rate': 600}],
    )
    # TODO for test
    message_pub_node = Node(
        package='roborts_test',
        executable='message_pub',
        output='screen',
        emulate_tty=True,
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen',
    )

    return LaunchDescription([
        message_pub_node,
        camera_node,
        detector_node,
        processor_node,
        robot_state_publisher,
        joint_state_publisher,
        # rviz2_node,
    ])
