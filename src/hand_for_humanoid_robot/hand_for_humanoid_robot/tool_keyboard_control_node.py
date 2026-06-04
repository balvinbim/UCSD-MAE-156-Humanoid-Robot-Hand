#!/usr/bin/env python3

import sys
import select
import termios
import tty
import time
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class ToolKeyboardControlNode(Node):
    def __init__(self):
        super().__init__('tool_keyboard_control_node')

        self.declare_parameter('roll_step_deg', 2.0)
        self.declare_parameter('max_roll_deg', 259.0)
        self.declare_parameter('grip_open_deg', 30.0)
        self.declare_parameter('grip_closed_deg', 0.0)
        self.declare_parameter('space_hold_timeout_sec', 0.35)
        self.declare_parameter('publish_rate_hz', 30.0)

        self.roll_step_deg = float(self.get_parameter('roll_step_deg').value)
        self.max_roll_deg = float(self.get_parameter('max_roll_deg').value)
        self.grip_open_deg = float(self.get_parameter('grip_open_deg').value)
        self.grip_closed_deg = float(self.get_parameter('grip_closed_deg').value)
        self.space_hold_timeout_sec = float(self.get_parameter('space_hold_timeout_sec').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.roll_deg = 0.0
        self.grip_deg = self.grip_closed_deg
        self.last_space_time = 0.0

        self.pub = self.create_publisher(
            JointState,
            '/h4hr/keyboard_tool_command',
            10
        )

        self.old_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.timer_callback
        )

        self.print_instructions()

    def print_instructions(self):
        print()
        print('H4HR Tool Keyboard Control')
        print('--------------------------')
        print('a = roll negative')
        print('d = roll positive')
        print('space = hold/repeat to open gripper')
        print('release space = gripper closes after timeout')
        print('r = reset roll to 0')
        print('q = quit keyboard node')
        print()
        print('Keep this terminal focused while using keyboard control.')
        print()

    def clamp(self, value, lower, upper):
        return max(lower, min(upper, value))

    def read_key_nonblocking(self):
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None

    def timer_callback(self):
        now = time.time()

        key = self.read_key_nonblocking()

        while key is not None:
            if key == 'a':
                self.roll_deg -= self.roll_step_deg

            elif key == 'd':
                self.roll_deg += self.roll_step_deg

            elif key == ' ':
                self.last_space_time = now

            elif key == 'r':
                self.roll_deg = 0.0

            elif key == 'q':
                self.get_logger().info('Quit requested from keyboard.')
                rclpy.shutdown()
                return

            self.roll_deg = self.clamp(
                self.roll_deg,
                -self.max_roll_deg,
                self.max_roll_deg
            )

            key = self.read_key_nonblocking()

        if now - self.last_space_time <= self.space_hold_timeout_sec:
            self.grip_deg = self.grip_open_deg
        else:
            self.grip_deg = self.grip_closed_deg

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'keyboard_roll',
            'keyboard_grip'
        ]
        msg.position = [
            math.radians(self.roll_deg),
            math.radians(self.grip_deg)
        ]

        self.pub.publish(msg)

        self.get_logger().info(
            f'Keyboard command deg: roll={self.roll_deg:.1f}, grip={self.grip_deg:.1f}'
        )

    def destroy_node(self):
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            self.old_terminal_settings
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ToolKeyboardControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
