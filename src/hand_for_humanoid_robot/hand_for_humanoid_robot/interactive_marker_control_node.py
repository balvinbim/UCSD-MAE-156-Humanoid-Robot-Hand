#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

from interactive_markers.interactive_marker_server import InteractiveMarkerServer


class InteractiveMarkerControlNode(Node):
    def __init__(self):
        super().__init__('interactive_marker_control_node')

        self.command_pub = self.create_publisher(
            JointState,
            '/h4hr/joint_command',
            10
        )

        self.server = InteractiveMarkerServer(
            self,
            'h4hr_interactive_marker'
        )

        # Current commanded joint values in degrees
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.grip_deg = 0.0

        # Joystick sensitivity
        # Lower these if movement is too aggressive.
        self.rotation_gain = 0.10
        self.grip_gain_deg_per_meter = 80.0

        # Command limits
        self.max_roll_deg = 259.0
        self.max_pitch_deg = 79.0
        self.max_yaw_deg = 79.0
        self.grip_min_deg = 0.0
        self.grip_max_deg = 30.0

        # Ignore tiny noise from RViz feedback
        self.rotation_deadband_deg = 0.15
        self.grip_deadband_m = 0.001

        self.marker_name = 'h4hr_relative_joystick_marker'

        # This prevents the reset motion from being interpreted as user input.
        self.ignore_next_feedback = False

        self.make_marker()

        self.publish_timer = self.create_timer(
            0.05,
            self.publish_joint_command
        )

        self.get_logger().info('Relative joystick interactive marker started.')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Blue/Z ring   -> roll only')
        self.get_logger().info('  Green/Y ring  -> pitch only')
        self.get_logger().info('  Red/X ring    -> yaw only')
        self.get_logger().info('  Up arrow      -> open gripper')
        self.get_logger().info('  Down arrow    -> close gripper')
        self.get_logger().info('Marker resets to neutral after each input.')

    def clamp(self, value, lower, upper):
        return max(lower, min(upper, value))

    def make_marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = 'base_link'
        marker.name = self.marker_name
        marker.description = 'Relative Joystick: blue=roll green=pitch red=yaw up/down=grip'
        marker.scale = 0.30

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        visual = Marker()
        visual.type = Marker.SPHERE
        visual.scale.x = 0.06
        visual.scale.y = 0.06
        visual.scale.z = 0.06
        visual.color.r = 0.1
        visual.color.g = 0.8
        visual.color.b = 1.0
        visual.color.a = 1.0

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(visual)
        marker.controls.append(visual_control)

        # Red / X ring = yaw
        yaw_control = InteractiveMarkerControl()
        yaw_control.name = 'yaw_red_x'
        yaw_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        yaw_control.orientation_mode = InteractiveMarkerControl.FIXED
        yaw_control.orientation.w = 1.0
        yaw_control.orientation.x = 1.0
        yaw_control.orientation.y = 0.0
        yaw_control.orientation.z = 0.0
        marker.controls.append(yaw_control)

        # Green / Y ring = pitch
        pitch_control = InteractiveMarkerControl()
        pitch_control.name = 'pitch_green_y'
        pitch_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        pitch_control.orientation_mode = InteractiveMarkerControl.FIXED
        pitch_control.orientation.w = 1.0
        pitch_control.orientation.x = 0.0
        pitch_control.orientation.y = 1.0
        pitch_control.orientation.z = 0.0
        marker.controls.append(pitch_control)

        # Blue / Z ring = roll
        roll_control = InteractiveMarkerControl()
        roll_control.name = 'roll_blue_z'
        roll_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        roll_control.orientation_mode = InteractiveMarkerControl.FIXED
        roll_control.orientation.w = 1.0
        roll_control.orientation.x = 0.0
        roll_control.orientation.y = 0.0
        roll_control.orientation.z = 1.0
        marker.controls.append(roll_control)

        # Up/down arrow = gripper
        grip_control = InteractiveMarkerControl()
        grip_control.name = 'grip_up_down_y'
        grip_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        grip_control.orientation_mode = InteractiveMarkerControl.FIXED
        grip_control.orientation.w = 1.0
        grip_control.orientation.x = 0.0
        grip_control.orientation.y = 1.0
        grip_control.orientation.z = 0.0
        marker.controls.append(grip_control)

        self.server.insert(marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def reset_marker_pose(self):
        stored_marker = self.server.get(self.marker_name)

        if stored_marker is None:
            return

        pose = stored_marker.pose

        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        self.ignore_next_feedback = True
        self.server.setPose(self.marker_name, pose)
        self.server.applyChanges()

    def quaternion_to_rpy(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)

        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def signed_axis_angle_deg(self, q, axis):
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        vector_norm = math.sqrt(x * x + y * y + z * z)

        if vector_norm < 1e-9:
            return 0.0

        angle_rad = 2.0 * math.atan2(vector_norm, w)

        if angle_rad > math.pi:
            angle_rad -= 2.0 * math.pi

        axis_x = x / vector_norm
        axis_y = y / vector_norm
        axis_z = z / vector_norm

        if axis == 'x':
            signed_angle_rad = angle_rad * axis_x
        elif axis == 'y':
            signed_angle_rad = angle_rad * axis_y
        elif axis == 'z':
            signed_angle_rad = angle_rad * axis_z
        else:
            signed_angle_rad = 0.0

        return math.degrees(signed_angle_rad)

    def process_feedback(self, feedback):
        if self.ignore_next_feedback:
            self.ignore_next_feedback = False
            return

        control_name = feedback.control_name
        did_update = False

        q = feedback.pose.orientation

        if control_name == 'roll_blue_z':
            delta_deg = self.signed_axis_angle_deg(q, 'z') * self.rotation_gain

            if abs(delta_deg) >= self.rotation_deadband_deg:
                self.roll_deg = self.clamp(
                    self.roll_deg + delta_deg,
                    -self.max_roll_deg,
                    self.max_roll_deg
                )
                did_update = True

        elif control_name == 'pitch_green_y':
            delta_deg = self.signed_axis_angle_deg(q, 'y') * self.rotation_gain

            if abs(delta_deg) >= self.rotation_deadband_deg:
                self.pitch_deg = self.clamp(
                    self.pitch_deg + delta_deg,
                    -self.max_pitch_deg,
                    self.max_pitch_deg
                )
                did_update = True

        elif control_name == 'yaw_red_x':
            delta_deg = self.signed_axis_angle_deg(q, 'x') * self.rotation_gain

            if abs(delta_deg) >= self.rotation_deadband_deg:
                self.yaw_deg = self.clamp(
                    self.yaw_deg + delta_deg,
                    -self.max_yaw_deg,
                    self.max_yaw_deg
                )
                did_update = True

        elif control_name == 'grip_up_down_y':
            y_motion = feedback.pose.position.y

            if abs(y_motion) >= self.grip_deadband_m:
                delta_grip = y_motion * self.grip_gain_deg_per_meter
                self.grip_deg = self.clamp(
                    self.grip_deg + delta_grip,
                    self.grip_min_deg,
                    self.grip_max_deg
                )
                did_update = True

        else:
            return

        if did_update:
            self.get_logger().info(
                f'Joystick command deg: '
                f'R={self.roll_deg:.1f}, '
                f'P={self.pitch_deg:.1f}, '
                f'Y={self.yaw_deg:.1f}, '
                f'G={self.grip_deg:.1f}'
            )

            # Keep this commented out for now so the marker does not crash/disappear.
            # self.reset_marker_pose()

    def publish_joint_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = [
            'roll_joint',
            'pitch_joint',
            'yaw_joint',
            'grip_joint',
        ]

        msg.position = [
            math.radians(self.roll_deg),
            math.radians(self.pitch_deg),
            math.radians(self.yaw_deg),
            math.radians(self.grip_deg),
        ]

        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = InteractiveMarkerControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()