#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    run_auto_zero = LaunchConfiguration('run_auto_zero')
    frame_id = LaunchConfiguration('frame_id')
    device_name = LaunchConfiguration('device_name')
    baudrate = LaunchConfiguration('baudrate')

    max_roll_deg = LaunchConfiguration('max_roll_deg')
    max_pitch_deg = LaunchConfiguration('max_pitch_deg')
    max_yaw_deg = LaunchConfiguration('max_yaw_deg')
    grip_deg = LaunchConfiguration('grip_deg')

    package_share_dir = get_package_share_directory('hand_for_humanoid_robot')
    rviz_config_file = os.path.join(
        package_share_dir,
        'rviz',
        'rviz_tip_control.rviz'
    )

    control_nodes = GroupAction([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'world',
                'base_link'
            ],
            output='screen'
        ),

        Node(
            package='hand_for_humanoid_robot',
            executable='rviz_tip_marker_node',
            name='rviz_tip_marker_node',
            output='screen',
            parameters=[
                {
                    'frame_id': frame_id,
                    'initial_x': 0.20,
                    'initial_y': 0.00,
                    'initial_z': 0.00,
                    'marker_scale': 0.20,
                    'publish_rate_hz': 20.0,
                }
            ]
        ),

        Node(
            package='hand_for_humanoid_robot',
            executable='tip_ik_node',
            name='tip_ik_node',
            output='screen',
            parameters=[
                {
                    'max_roll_deg': max_roll_deg,
                    'max_pitch_deg': max_pitch_deg,
                    'max_yaw_deg': max_yaw_deg,
                    'grip_deg': grip_deg,

                    'use_pose_roll': True,
                    'fixed_roll_deg': 0.0,

                    'base_x': 0.0,
                    'base_y': 0.0,
                    'base_z': 0.0,

                    'deadzone_m': 0.005,
                }
            ]
        ),

        Node(
            package='hand_for_humanoid_robot',
            executable='joint_command_bridge_node',
            name='joint_command_bridge_node',
            output='screen',
            parameters=[
                {
                    'max_roll_deg': max_roll_deg,
                    'max_pitch_deg': max_pitch_deg,
                    'max_yaw_deg': max_yaw_deg,
                    'min_grip_deg': 0.0,
                    'max_grip_deg': 30.0,
                }
            ]
        ),

        Node(
            package='hand_for_humanoid_robot',
            executable='dynamixel_controller_node',
            name='dynamixel_controller_node',
            output='screen',
            parameters=[
                {
                    'device_name': device_name,
                    'baudrate': baudrate,
                }
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_rviz)
        ),
    ])

    auto_zero_node = Node(
        package='hand_for_humanoid_robot',
        executable='auto_zero_node',
        name='auto_zero_node',
        output='screen',
        condition=IfCondition(run_auto_zero),
        parameters=[
            {
                'device_name': device_name,
                'baudrate': baudrate,

                'exit_after_sequence': True,
                'torque_off_on_shutdown': True,
            }
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz automatically'
        ),

        DeclareLaunchArgument(
            'run_auto_zero',
            default_value='true',
            description='Run auto-zero before starting RViz tool control'
        ),

        DeclareLaunchArgument(
            'device_name',
            default_value='/dev/ttyACM0',
            description='OpenRB/Dynamixel serial device'
        ),

        DeclareLaunchArgument(
            'baudrate',
            default_value='57600',
            description='Dynamixel baudrate'
        ),

        DeclareLaunchArgument(
            'frame_id',
            default_value='base_link',
            description='Frame used by RViz tip marker'
        ),

        DeclareLaunchArgument(
            'max_roll_deg',
            default_value='259.0',
            description='Maximum allowed roll command in degrees'
        ),

        DeclareLaunchArgument(
            'max_pitch_deg',
            default_value='79.0',
            description='Maximum allowed pitch command in degrees'
        ),

        DeclareLaunchArgument(
            'max_yaw_deg',
            default_value='79.0',
            description='Maximum allowed yaw command in degrees'
        ),

        DeclareLaunchArgument(
            'grip_deg',
            default_value='0.0',
            description='Fixed gripper command in degrees'
        ),

        auto_zero_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=auto_zero_node,
                on_exit=[
                    control_nodes
                ]
            ),
            condition=IfCondition(run_auto_zero)
        ),

        GroupAction(
            [
                control_nodes
            ],
            condition=UnlessCondition(run_auto_zero)
        ),
    ])