#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class TipIKNode(Node):
    def __init__(self):
        super().__init__('tip_ik_node')

        # Tool base location in the RViz/ROS frame.
        self.declare_parameter('base_x', 0.0)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('base_z', 0.0)

        # Safety limits.
        self.declare_parameter('max_roll_deg', 45.0)
        self.declare_parameter('max_pitch_deg', 45.0)
        self.declare_parameter('max_yaw_deg', 45.0)
        self.declare_parameter('grip_deg', 0.0)

        # If True, use roll from /desired_tip_pose orientation.
        # If False, use fixed_roll_deg.
        self.declare_parameter('use_pose_roll', True)
        self.declare_parameter('fixed_roll_deg', 0.0)

        # Deadzone prevents weird behavior if target is too close to base.
        self.declare_parameter('deadzone_m', 0.005)

        self.base_x = float(self.get_parameter('base_x').value)
        self.base_y = float(self.get_parameter('base_y').value)
        self.base_z = float(self.get_parameter('base_z').value)

        self.max_roll_deg = float(self.get_parameter('max_roll_deg').value)
        self.max_pitch_deg = float(self.get_parameter('max_pitch_deg').value)
        self.max_yaw_deg = float(self.get_parameter('max_yaw_deg').value)

        self.grip_deg = float(self.get_parameter('grip_deg').value)
        self.use_pose_roll = bool(self.get_parameter('use_pose_roll').value)
        self.fixed_roll_deg = float(self.get_parameter('fixed_roll_deg').value)
        self.deadzone_m = float(self.get_parameter('deadzone_m').value)

        self.sub = self.create_subscription(
            PoseStamped,
            '/desired_tip_pose',
            self.tip_callback,
            10
        )

        self.pub = self.create_publisher(
            JointState,
            '/target_tool_joints',
            10
        )

        self.get_logger().info('tip_ik_node started.')
        self.get_logger().info('Listening to /desired_tip_pose')
        self.get_logger().info('Publishing to /target_tool_joints')
        self.get_logger().info(f'use_pose_roll = {self.use_pose_roll}')
        self.get_logger().info(f'fixed_roll_deg = {self.fixed_roll_deg:.1f}')

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def quaternion_to_roll_deg(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)

        roll_rad = math.atan2(sinr_cosp, cosr_cosp)
        return math.degrees(roll_rad)

    def tip_callback(self, msg):
        x_tip = msg.pose.position.x
        y_tip = msg.pose.position.y
        z_tip = msg.pose.position.z

        dx = x_tip - self.base_x
        dy = y_tip - self.base_y
        dz = z_tip - self.base_z

        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance < self.deadzone_m:
            roll_deg = 0.0
            pitch_deg = 0.0
            yaw_deg = 0.0
            grip_deg = 0.0

            self.get_logger().warn(
                'Desired tip point is too close to tool base. Publishing zero command.'
            )
        else:
            horizontal_distance = math.sqrt(dx * dx + dy * dy)

            # Assumption:
            # +X = forward along the tool shaft
            # +Y = left
            # +Z = up
            yaw_rad = math.atan2(dy, dx)
            pitch_rad = math.atan2(dz, horizontal_distance)

            yaw_deg = math.degrees(yaw_rad)
            pitch_deg = math.degrees(pitch_rad)

            if self.use_pose_roll:
                roll_deg = self.quaternion_to_roll_deg(msg.pose.orientation)
            else:
                roll_deg = self.fixed_roll_deg

            roll_deg = self.clamp(
                roll_deg,
                -self.max_roll_deg,
                self.max_roll_deg
            )

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

            grip_deg = self.grip_deg

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()

        joint_msg.name = [
            'tool_roll',
            'tool_pitch',
            'tool_yaw',
            'tool_grip'
        ]

        # JointState uses radians.
        joint_msg.position = [
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
            math.radians(grip_deg)
        ]

        self.pub.publish(joint_msg)

        self.get_logger().info(
            f'Input tip: x={x_tip:.3f}, y={y_tip:.3f}, z={z_tip:.3f} m | '
            f'IK output: roll={roll_deg:.1f} deg, '
            f'pitch={pitch_deg:.1f} deg, '
            f'yaw={yaw_deg:.1f} deg, '
            f'grip={grip_deg:.1f} deg'
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
