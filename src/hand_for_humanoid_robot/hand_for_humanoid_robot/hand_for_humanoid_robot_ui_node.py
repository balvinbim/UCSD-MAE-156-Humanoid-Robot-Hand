#!/usr/bin/env python3

import sys
import select
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState


class HandForHumanoidRobotUiNode(Node):
    def __init__(self):
        super().__init__('hand_for_humanoid_robot_ui_node')

        self.no_tool_pub = self.create_publisher(
            Bool,
            '/h4hr/confirm_no_tool',
            10
        )

        self.tool_inserted_pub = self.create_publisher(
            Bool,
            '/h4hr/confirm_tool_insertion',
            10
        )

        # Resets RViz/RViz-control command topic.
        self.marker_reset_pub = self.create_publisher(
            JointState,
            '/target_tool_joints',
            10
        )

        # Sends actual motor command back to zero through the controller path.
        self.motor_reset_pub = self.create_publisher(
            JointState,
            '/h4hr/joint_command',
            10
        )

        self.get_logger().info('Hand for Humanoid Robot UI node started.')

        self.print_menu()

        self.timer = self.create_timer(0.1, self.check_input)

    def print_menu(self):
        print()
        print('===================================')
        print(' Hand for Humanoid Robot UI')
        print('===================================')
        print('  n  - Confirm NO tool inserted')
        print('  y  - Confirm tool IS inserted')
        print('  r  - Reset RViz command AND motors to home')
        print('  ?  - Show menu')
        print('  q  - Quit UI')
        print('===================================')
        print()

    def publish_bool(self, publisher, value=True):
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

    def check_input(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)

        if not readable:
            return

        line = sys.stdin.readline().strip().lower()

        if line == 'n':
            self.publish_bool(self.no_tool_pub, True)
            self.get_logger().info('Published: confirm NO tool inserted.')

        elif line == 'y':
            self.publish_bool(self.tool_inserted_pub, True)
            self.get_logger().info('Published: confirm tool inserted.')

        elif line == 'r':
            self.reset_to_home()

        elif line == '?':
            self.print_menu()

        elif line == 'q':
            self.get_logger().info('Quitting UI node.')
            rclpy.shutdown()

        else:
            print(f'Unknown command: "{line}". Type ? for menu.')

    def reset_to_home(self):
        # Topic used by RViz orientation control node.
        rviz_msg = JointState()
        rviz_msg.header.stamp = self.get_clock().now().to_msg()

        rviz_msg.name = [
            'tool_roll',
            'tool_pitch',
            'tool_yaw',
            'tool_grip',
        ]

        rviz_msg.position = [
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
        ]

        # Topic used by Dynamixel controller.
        motor_msg = JointState()
        motor_msg.header.stamp = self.get_clock().now().to_msg()

        motor_msg.name = [
            'roll_joint',
            'pitch_joint',
            'yaw_joint',
            'grip_joint',
        ]

        motor_msg.position = [
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0),
        ]

        # Publish several times so subscribers reliably receive the reset.
        for _ in range(5):
            self.marker_reset_pub.publish(rviz_msg)
            self.motor_reset_pub.publish(motor_msg)

        self.get_logger().info(
            'Published reset: RViz command and motor command set to R=0, P=0, Y=0, G=0.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = HandForHumanoidRobotUiNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()