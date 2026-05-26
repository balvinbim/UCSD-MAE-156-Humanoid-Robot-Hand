from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('hand_for_humanoid_robot')
    rviz_config = os.path.join(package_share, 'rviz', 'h4hr_control.rviz')

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen'
    )

    dynamixel_controller_node = Node(
        package='hand_for_humanoid_robot',
        executable='dynamixel_controller_node',
        name='dynamixel_controller_node',
        output='screen'
    )

    interactive_marker_node = Node(
        package='hand_for_humanoid_robot',
        executable='interactive_marker_control_node',
        name='interactive_marker_control_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe',
        }
    )

    return LaunchDescription([
        static_tf_node,
        dynamixel_controller_node,
        interactive_marker_node,
        rviz_node,
    ])