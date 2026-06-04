#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

from interactive_markers.interactive_marker_server import InteractiveMarkerServer


class RvizOrientationControlNode(Node):
    def __init__(self):
        super().__init__('rviz_orientation_control_node')

        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('initial_x', 0.00)
        self.declare_parameter('initial_y', 0.00)
        self.declare_parameter('initial_z', 0.00)
        self.declare_parameter('marker_scale', 0.35)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.declare_parameter('max_roll_deg', 259.0)
        self.declare_parameter('max_pitch_deg', 79.0)
        self.declare_parameter('max_yaw_deg', 79.0)
        self.declare_parameter('max_grip_deg', 30.0)

        self.declare_parameter('grip_marker_min_x', -0.20)
        self.declare_parameter('grip_marker_max_x', 0.20)
        self.declare_parameter('grip_marker_y', -0.35)
        self.declare_parameter('grip_marker_z', 0.00)

        self.declare_parameter('keyboard_timeout_sec', 1.0)
        self.declare_parameter('input_rotation_full_scale_deg', 360.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_z = float(self.get_parameter('initial_z').value)
        self.marker_scale = float(self.get_parameter('marker_scale').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.max_roll_deg = float(self.get_parameter('max_roll_deg').value)
        self.max_pitch_deg = float(self.get_parameter('max_pitch_deg').value)
        self.max_yaw_deg = float(self.get_parameter('max_yaw_deg').value)
        self.max_grip_deg = float(self.get_parameter('max_grip_deg').value)

        self.grip_marker_min_x = float(self.get_parameter('grip_marker_min_x').value)
        self.grip_marker_max_x = float(self.get_parameter('grip_marker_max_x').value)
        self.grip_marker_y = float(self.get_parameter('grip_marker_y').value)
        self.grip_marker_z = float(self.get_parameter('grip_marker_z').value)

        self.keyboard_timeout_sec = float(self.get_parameter('keyboard_timeout_sec').value)
        self.input_rotation_full_scale_deg = float(
            self.get_parameter('input_rotation_full_scale_deg').value
        )

        self.marker_name = 'dvrk_orientation_cube'
        self.grip_marker_name = 'dvrk_gripper_slider'

        # Final accumulated command values in degrees.
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.grip_deg = 0.0

        # Previous quaternion position for free cube rotations
        self.prev_raw_quat = None

        # Previous raw angles for individual rings.
        self.prev_ring_roll_quat = None
        self.prev_ring_pitch_quat = None
        self.prev_ring_yaw_quat = None

        self.last_keyboard_msg_time = 0.0
        self.keyboard_grip_deg = 0.0

        self.server = InteractiveMarkerServer(
            self,
            'dvrk_orientation_marker_server'
        )

        self.joint_pub = self.create_publisher(
            JointState,
            '/target_tool_joints',
            10
        )

        self.keyboard_sub = self.create_subscription(
            JointState,
            '/h4hr/keyboard_tool_command',
            self.keyboard_callback,
            10
        )

        self.make_marker()
        self.make_gripper_marker()

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_tool_joints
        )

        self.get_logger().info('rviz_orientation_control_node started.')
        self.get_logger().info('Cube controls combined roll/pitch/yaw.')
        self.get_logger().info('Rings control one joint only.')
        self.get_logger().info('Keyboard space controls grip through /h4hr/keyboard_tool_command.')
        self.get_logger().info('In RViz, add InteractiveMarkers topic: /dvrk_orientation_marker_server/update')

    def clamp(self, value, lower, upper):
        return max(lower, min(upper, value))

    def wrapped_delta_deg(self, current_deg, previous_deg):
        if previous_deg is None:
            return 0.0

        return (current_deg - previous_deg + 180.0) % 360.0 - 180.0

    def apply_delta_with_limit(self, current_value, delta_value, lower_limit, upper_limit):
        if current_value >= upper_limit and delta_value > 0.0:
            return upper_limit

        if current_value <= lower_limit and delta_value < 0.0:
            return lower_limit

        return self.clamp(
            current_value + delta_value,
            lower_limit,
            upper_limit
        )

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

    def make_marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = self.marker_name
        marker.description = 'Cube = free R/P/Y, rings = one-axis R/P/Y'
        marker.scale = self.marker_scale

        marker.pose.position.x = self.initial_x
        marker.pose.position.y = self.initial_y
        marker.pose.position.z = self.initial_z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = 0.1
        cube.scale.y = 0.1
        cube.scale.z = 0.1
        cube.color.r = 0.2
        cube.color.g = 0.8
        cube.color.b = 1.0
        cube.color.a = 1.0

        cube.pose.position.x = 0.0
        cube.pose.position.y = 0.0
        cube.pose.position.z = 0.0
        cube.pose.orientation.x = 0.0
        cube.pose.orientation.y = 0.0
        cube.pose.orientation.z = 0.0
        cube.pose.orientation.w = 1.0

        cube_control = InteractiveMarkerControl()
        cube_control.name = 'rotate_3d_cube'
        cube_control.interaction_mode = InteractiveMarkerControl.ROTATE_3D
        cube_control.orientation_mode = InteractiveMarkerControl.INHERIT
        cube_control.always_visible = True
        cube_control.markers.append(cube)
        marker.controls.append(cube_control)

        roll_control = InteractiveMarkerControl()
        roll_control.name = 'rotate_x_roll_ring'
        roll_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        roll_control.orientation_mode = InteractiveMarkerControl.FIXED
        roll_control.orientation.w = 1.0
        roll_control.orientation.x = 1.0
        roll_control.orientation.y = 0.0
        roll_control.orientation.z = 0.0
        marker.controls.append(roll_control)

        pitch_control = InteractiveMarkerControl()
        pitch_control.name = 'rotate_y_pitch_ring'
        pitch_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        pitch_control.orientation_mode = InteractiveMarkerControl.FIXED
        pitch_control.orientation.w = 1.0
        pitch_control.orientation.x = 0.0
        pitch_control.orientation.y = 1.0
        pitch_control.orientation.z = 0.0
        marker.controls.append(pitch_control)

        yaw_control = InteractiveMarkerControl()
        yaw_control.name = 'rotate_z_yaw_ring'
        yaw_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        yaw_control.orientation_mode = InteractiveMarkerControl.FIXED
        yaw_control.orientation.w = 1.0
        yaw_control.orientation.x = 0.0
        yaw_control.orientation.y = 0.0
        yaw_control.orientation.z = 1.0
        marker.controls.append(yaw_control)

        self.server.insert(marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def make_gripper_marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = self.grip_marker_name
        marker.description = 'Drag right = open gripper'
        marker.scale = 0.25

        marker.pose.position.x = self.grip_marker_min_x
        marker.pose.position.y = self.grip_marker_y
        marker.pose.position.z = self.grip_marker_z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.scale.x = 0.18
        arrow.scale.y = 0.035
        arrow.scale.z = 0.035
        arrow.color.r = 1.0
        arrow.color.g = 0.6
        arrow.color.b = 0.1
        arrow.color.a = 1.0

        arrow.pose.position.x = 0.0
        arrow.pose.position.y = 0.0
        arrow.pose.position.z = 0.0
        arrow.pose.orientation.x = 0.0
        arrow.pose.orientation.y = 0.0
        arrow.pose.orientation.z = 0.0
        arrow.pose.orientation.w = 1.0

        arrow_control = InteractiveMarkerControl()
        arrow_control.name = 'gripper_slider_arrow'
        arrow_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        arrow_control.orientation_mode = InteractiveMarkerControl.FIXED

        # X-axis movement.
        arrow_control.orientation.w = 1.0
        arrow_control.orientation.x = 1.0
        arrow_control.orientation.y = 0.0
        arrow_control.orientation.z = 0.0

        arrow_control.always_visible = True
        arrow_control.markers.append(arrow)
        marker.controls.append(arrow_control)

        self.server.insert(marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
        )

    def quaternion_to_rpy_deg(self, q):
        x, y, z, w = q
        sinr_cosp = 2.0 * (w*x + y*z)
        cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
        roll_rad  = math.atan2(sinr_cosp, cosr_cosp)

        sinp = max(-1.0, min(1.0, 2.0 * (w*y - z*x)))
        pitch_rad = math.asin(sinp)

        siny_cosp = 2.0 * (w*z + x*y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        yaw_rad   = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(roll_rad), math.degrees(pitch_rad), math.degrees(yaw_rad)

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if feedback.control_name == 'rotate_x_roll_ring':
                self.prev_ring_roll_quat = None
            elif feedback.control_name == 'rotate_y_pitch_ring':
                self.prev_ring_pitch_quat = None
            elif feedback.control_name == 'rotate_z_yaw_ring':
                self.prev_ring_yaw_quat = None
            elif feedback.control_name == 'rotate_3d_cube':
                self.prev_raw_quat = None

            return

        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return

        control_name = feedback.control_name

        if feedback.marker_name == self.grip_marker_name:
            x = self.clamp(
                feedback.pose.position.x,
                self.grip_marker_min_x,
                self.grip_marker_max_x
            )

            feedback.pose.position.x = x
            feedback.pose.position.y = self.grip_marker_y
            feedback.pose.position.z = self.grip_marker_z

            self.server.setPose(self.grip_marker_name, feedback.pose)
            self.server.applyChanges()

            travel = self.grip_marker_max_x - self.grip_marker_min_x

            if travel <= 0.0:
                grip_ratio = 0.0
            else:
                grip_ratio = (x - self.grip_marker_min_x) / travel

            self.grip_deg = self.clamp(
                grip_ratio * self.max_grip_deg,
                0.0,
                self.max_grip_deg
            )

            self.get_logger().info(
                f'Gripper slider: x={x:.3f}, grip={self.grip_deg:.1f} deg'
            )

            return

        self.get_logger().info(f'Control used: {control_name}')

        if control_name == 'rotate_3d_cube':
            self.server.setPose(self.marker_name, feedback.pose)
            self.server.applyChanges()

        roll_scale = self.max_roll_deg / self.input_rotation_full_scale_deg
        pitch_scale = self.max_pitch_deg / self.input_rotation_full_scale_deg
        yaw_scale = self.max_yaw_deg / self.input_rotation_full_scale_deg

        if control_name == 'rotate_x_roll_ring':
            q_curr = (
                feedback.pose.orientation.x,
                feedback.pose.orientation.y,
                feedback.pose.orientation.z,
                feedback.pose.orientation.w,
            )
            if self.prev_ring_roll_quat is None:
                self.prev_ring_roll_quat = q_curr
                return
            px, py, pz, pw = self.prev_ring_roll_quat
            q_delta = self.quaternion_multiply((-px, -py, -pz, pw), q_curr)
            dx, dy, dz, dw = q_delta
            n = math.sqrt(dx*dx + dy*dy + dz*dz + dw*dw)
            q_delta = (dx/n, dy/n, dz/n, dw/n)
            delta_roll_deg, _, _ = self.quaternion_to_rpy_deg(q_delta)
            self.prev_ring_roll_quat = q_curr
            self.roll_deg = self.apply_delta_with_limit(
                self.roll_deg, delta_roll_deg * roll_scale,
                -self.max_roll_deg, self.max_roll_deg
            )

        elif control_name == 'rotate_y_pitch_ring':
            q_curr = (
                feedback.pose.orientation.x,
                feedback.pose.orientation.y,
                feedback.pose.orientation.z,
                feedback.pose.orientation.w,
            )
            if self.prev_ring_pitch_quat is None:
                self.prev_ring_pitch_quat = q_curr
                return
            px, py, pz, pw = self.prev_ring_pitch_quat
            q_delta = self.quaternion_multiply((-px, -py, -pz, pw), q_curr)
            dx, dy, dz, dw = q_delta
            n = math.sqrt(dx*dx + dy*dy + dz*dz + dw*dw)
            q_delta = (dx/n, dy/n, dz/n, dw/n)
            _, delta_pitch_deg, _ = self.quaternion_to_rpy_deg(q_delta)
            self.prev_ring_pitch_quat = q_curr
            self.pitch_deg = self.apply_delta_with_limit(
                self.pitch_deg, delta_pitch_deg * pitch_scale,
                -self.max_pitch_deg, self.max_pitch_deg
            )

        elif control_name == 'rotate_z_yaw_ring':
            q_curr = (
                feedback.pose.orientation.x,
                feedback.pose.orientation.y,
                feedback.pose.orientation.z,
                feedback.pose.orientation.w,
            )
            if self.prev_ring_yaw_quat is None:
                self.prev_ring_yaw_quat = q_curr
                return
            px, py, pz, pw = self.prev_ring_yaw_quat
            q_delta = self.quaternion_multiply((-px, -py, -pz, pw), q_curr)
            dx, dy, dz, dw = q_delta
            n = math.sqrt(dx*dx + dy*dy + dz*dz + dw*dw)
            q_delta = (dx/n, dy/n, dz/n, dw/n)
            _, _, delta_yaw_deg = self.quaternion_to_rpy_deg(q_delta)
            self.prev_ring_yaw_quat = q_curr
            self.yaw_deg = self.apply_delta_with_limit(
                self.yaw_deg, delta_yaw_deg * yaw_scale,
                -self.max_yaw_deg, self.max_yaw_deg
            )

        elif control_name == 'rotate_3d_cube':
            q_curr = (
                feedback.pose.orientation.x,
                feedback.pose.orientation.y,
                feedback.pose.orientation.z,
                feedback.pose.orientation.w,
            )

            if self.prev_raw_quat is None:
                self.prev_raw_quat = q_curr
                return

            px, py, pz, pw = self.prev_raw_quat
            q_prev_conj = (-px, -py, -pz, pw)
            q_delta = self.quaternion_multiply(q_prev_conj, q_curr)

            dx, dy, dz, dw = q_delta
            n = math.sqrt(dx*dx + dy*dy + dz*dz + dw*dw)
            q_delta = (dx/n, dy/n, dz/n, dw/n)

            delta_roll_deg, delta_pitch_deg, delta_yaw_deg = self.quaternion_to_rpy_deg(q_delta)

            self.prev_raw_quat = q_curr

            self.roll_deg = self.apply_delta_with_limit(
                self.roll_deg, delta_roll_deg * roll_scale,
                -self.max_roll_deg, self.max_roll_deg
            )
            self.pitch_deg = self.apply_delta_with_limit(
                self.pitch_deg, delta_pitch_deg * pitch_scale,
                -self.max_pitch_deg, self.max_pitch_deg
            )
            self.yaw_deg = self.apply_delta_with_limit(
                self.yaw_deg, delta_yaw_deg * yaw_scale,
                -self.max_yaw_deg, self.max_yaw_deg
            )

    def keyboard_callback(self, msg):
        if len(msg.position) < 2:
            return

        self.keyboard_grip_deg = math.degrees(msg.position[1])
        self.last_keyboard_msg_time = time.time()

    def mouse_angle_about_z_deg(self, feedback):
        dx = feedback.mouse_point.x - feedback.pose.position.x
        dy = feedback.mouse_point.y - feedback.pose.position.y

        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return None

        return math.degrees(math.atan2(dy, dx))

    def get_grip_deg(self):
        now = time.time()

        keyboard_fresh = (
            self.last_keyboard_msg_time > 0.0 and
            now - self.last_keyboard_msg_time <= self.keyboard_timeout_sec
        )

        if keyboard_fresh:
            grip_deg = self.keyboard_grip_deg
        else:
            grip_deg = self.grip_deg

        return self.clamp(
            grip_deg,
            0.0,
            self.max_grip_deg
        )

    def publish_tool_joints(self):
        self.grip_deg = self.get_grip_deg()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = [
            'tool_roll',
            'tool_pitch',
            'tool_yaw',
            'tool_grip'
        ]

        msg.position = [
            math.radians(self.roll_deg),
            math.radians(self.pitch_deg),
            math.radians(self.yaw_deg),
            math.radians(self.grip_deg)
        ]

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RvizOrientationControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()