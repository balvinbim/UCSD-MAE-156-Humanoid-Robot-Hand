#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class TipIKNode(Node):
    def __init__(self):
        super().__init__('tip_ik_node')

        self.declare_parameter('base_x', 0.0)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('base_z', 0.0)

        self.declare_parameter('max_roll_deg', 259.0)
        self.declare_parameter('max_pitch_deg', 79.0)
        self.declare_parameter('max_yaw_deg', 79.0)
        self.declare_parameter('max_grip_deg', 30.0)

        self.declare_parameter('use_keyboard_commands', True)
        self.declare_parameter('keyboard_timeout_sec', 1.0)

        self.declare_parameter('fixed_roll_deg', 0.0)
        self.declare_parameter('fixed_grip_deg', 0.0)

        self.declare_parameter('deadzone_m', 0.005)

        self.base_x = float(self.get_parameter('base_x').value)
        self.base_y = float(self.get_parameter('base_y').value)
        self.base_z = float(self.get_parameter('base_z').value)

        self.max_roll_deg = float(self.get_parameter('max_roll_deg').value)
        self.max_pitch_deg = float(self.get_parameter('max_pitch_deg').value)
        self.max_yaw_deg = float(self.get_parameter('max_yaw_deg').value)
        self.max_grip_deg = float(self.get_parameter('max_grip_deg').value)

        self.use_keyboard_commands = bool(self.get_parameter('use_keyboard_commands').value)
        self.keyboard_timeout_sec = float(self.get_parameter('keyboard_timeout_sec').value)

        self.fixed_roll_deg = float(self.get_parameter('fixed_roll_deg').value)
        self.fixed_grip_deg = float(self.get_parameter('fixed_grip_deg').value)

        self.deadzone_m = float(self.get_parameter('deadzone_m').value)

        self.keyboard_roll_deg = 0.0
        self.keyboard_grip_deg = 0.0
        self.last_keyboard_msg_time = 0.0

        self.tip_sub = self.create_subscription(
            PoseStamped,
            '/desired_tip_pose',
            self.tip_callback,
            10
        )

        self.keyboard_sub = self.create_subscription(
            JointState,
            '/h4hr/keyboard_tool_command',
            self.keyboard_callback,
            10
        )

        self.pub = self.create_publisher(
            JointState,
            '/target_tool_joints',
            10
        )

        self.get_logger().info('tip_ik_node started.')
        self.get_logger().info('RViz /desired_tip_pose controls pitch/yaw.')
        self.get_logger().info('Keyboard /h4hr/keyboard_tool_command controls roll/grip.')
        self.get_logger().info(f'use_keyboard_commands = {self.use_keyboard_commands}')

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def keyboard_callback(self, msg):
        if len(msg.position) < 2:
            self.get_logger().warn(
                'Received /h4hr/keyboard_tool_command with fewer than 2 positions.'
            )
            return

        self.keyboard_roll_deg = math.degrees(msg.position[0])
        self.keyboard_grip_deg = math.degrees(msg.position[1])
        self.last_keyboard_msg_time = time.time()

    def get_roll_and_grip_deg(self):
        now = time.time()

        keyboard_is_fresh = (
            self.last_keyboard_msg_time > 0.0 and
            now - self.last_keyboard_msg_time <= self.keyboard_timeout_sec
        )

        if self.use_keyboard_commands and keyboard_is_fresh:
            roll_deg = self.keyboard_roll_deg
            grip_deg = self.keyboard_grip_deg
        else:
            roll_deg = self.fixed_roll_deg
            grip_deg = self.fixed_grip_deg

        roll_deg = self.clamp(
            roll_deg,
            -self.max_roll_deg,
            self.max_roll_deg
        )

        grip_deg = self.clamp(
            grip_deg,
            0.0,
            self.max_grip_deg
        )

        return roll_deg, grip_deg

    def tip_callback(self, msg):
        x_tip = msg.pose.position.x
        y_tip = msg.pose.position.y
        z_tip = msg.pose.position.z

        dx = x_tip - self.base_x
        dy = y_tip - self.base_y
        dz = z_tip - self.base_z

        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance < self.deadzone_m:
            pitch_deg = 0.0
            yaw_deg = 0.0

            self.get_logger().warn(
                'Desired tip point is too close to tool base. Pitch/yaw set to zero.'
            )
        else:
            horizontal_distance = math.sqrt(dx * dx + dy * dy)

            yaw_rad = math.atan2(dy, dx)
            pitch_rad = math.atan2(dz, horizontal_distance)

            yaw_deg = math.degrees(yaw_rad)
            pitch_deg = math.degrees(pitch_rad)

            pitch_deg = self.clamp(
                pitch_deg,
                -self.max_pitch_deg,
                self.max_pitch_deg
            )

            yaw_deg = self.clamp(
                yaw_deg,
                -self.max_yaw_deg,
                self.max_yaw_deg
            )

        roll_deg, grip_deg = self.get_roll_and_grip_deg()

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()

        joint_msg.name = [
            'tool_roll',
            'tool_pitch',
            'tool_yaw',
            'tool_grip'
        ]

        joint_msg.position = [
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
            math.radians(grip_deg)
        ]

        self.pub.publish(joint_msg)

        self.get_logger().info(
            f'Input tip: x={x_tip:.3f}, y={y_tip:.3f}, z={z_tip:.3f} m | '
            f'Output deg: roll={roll_deg:.1f}, '
            f'pitch={pitch_deg:.1f}, '
            f'yaw={yaw_deg:.1f}, '
            f'grip={grip_deg:.1f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TipIKNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()