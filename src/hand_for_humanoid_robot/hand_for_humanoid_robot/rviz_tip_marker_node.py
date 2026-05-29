#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

from interactive_markers.interactive_marker_server import InteractiveMarkerServer


class RvizTipMarkerNode(Node):
    def __init__(self):
        super().__init__('rviz_tip_marker_node')

        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('initial_x', 0.20)
        self.declare_parameter('initial_y', 0.00)
        self.declare_parameter('initial_z', 0.00)
        self.declare_parameter('marker_scale', 0.20)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_z = float(self.get_parameter('initial_z').value)
        self.marker_scale = float(self.get_parameter('marker_scale').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.marker_name = 'desired_dvrk_tip_marker'

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/desired_tip_pose',
            10
        )

        self.server = InteractiveMarkerServer(
            self,
            'desired_tip_marker_server'
        )

        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = self.frame_id
        self.current_pose.pose.position.x = self.initial_x
        self.current_pose.pose.position.y = self.initial_y
        self.current_pose.pose.position.z = self.initial_z
        self.current_pose.pose.orientation.x = 0.0
        self.current_pose.pose.orientation.y = 0.0
        self.current_pose.pose.orientation.z = 0.0
        self.current_pose.pose.orientation.w = 1.0

        self.make_marker()

        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(
            timer_period,
            self.publish_desired_tip_pose
        )

        self.get_logger().info('rviz_tip_marker_node started.')
        self.get_logger().info(f'Publishing /desired_tip_pose in frame: {self.frame_id}')
        self.get_logger().info('Move the sphere in RViz to command desired tip position.')
        self.get_logger().info('Use the red roll ring if you want to command roll orientation.')

    def make_marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = self.marker_name
        marker.description = 'Desired dVRK Tip Pose'
        marker.scale = self.marker_scale

        marker.pose.position.x = self.initial_x
        marker.pose.position.y = self.initial_y
        marker.pose.position.z = self.initial_z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.035
        sphere.scale.y = 0.035
        sphere.scale.z = 0.035
        sphere.color.r = 0.0
        sphere.color.g = 1.0
        sphere.color.b = 0.2
        sphere.color.a = 1.0

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(sphere)
        marker.controls.append(visual_control)

        move_x = InteractiveMarkerControl()
        move_x.name = 'move_x'
        move_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move_x.orientation_mode = InteractiveMarkerControl.FIXED
        move_x.orientation.w = 1.0
        move_x.orientation.x = 1.0
        move_x.orientation.y = 0.0
        move_x.orientation.z = 0.0
        marker.controls.append(move_x)

        move_y = InteractiveMarkerControl()
        move_y.name = 'move_y'
        move_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move_y.orientation_mode = InteractiveMarkerControl.FIXED
        move_y.orientation.w = 1.0
        move_y.orientation.x = 0.0
        move_y.orientation.y = 1.0
        move_y.orientation.z = 0.0
        marker.controls.append(move_y)

        move_z = InteractiveMarkerControl()
        move_z.name = 'move_z'
        move_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move_z.orientation_mode = InteractiveMarkerControl.FIXED
        move_z.orientation.w = 1.0
        move_z.orientation.x = 0.0
        move_z.orientation.y = 0.0
        move_z.orientation.z = 1.0
        marker.controls.append(move_z)

        move_3d = InteractiveMarkerControl()
        move_3d.name = 'move_3d'
        move_3d.interaction_mode = InteractiveMarkerControl.MOVE_3D
        move_3d.orientation_mode = InteractiveMarkerControl.FIXED
        move_3d.always_visible = False
        marker.controls.append(move_3d)

        # Optional roll command ring.
        # This lets the marker orientation carry a roll command.
        roll_control = InteractiveMarkerControl()
        roll_control.name = 'roll_about_x'
        roll_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        roll_control.orientation_mode = InteractiveMarkerControl.FIXED
        roll_control.orientation.w = 1.0
        roll_control.orientation.x = 1.0
        roll_control.orientation.y = 0.0
        roll_control.orientation.z = 0.0
        marker.controls.append(roll_control)

        self.server.insert(marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        if feedback.event_type not in [
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP
        ]:
            return

        self.current_pose.header.frame_id = self.frame_id
        self.current_pose.pose = feedback.pose

        self.get_logger().info(
            f'Desired tip marker: '
            f'x={feedback.pose.position.x:.3f}, '
            f'y={feedback.pose.position.y:.3f}, '
            f'z={feedback.pose.position.z:.3f}'
        )

    def publish_desired_tip_pose(self):
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.current_pose)


def main(args=None):
    rclpy.init(args=args)
    node = RvizTipMarkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
