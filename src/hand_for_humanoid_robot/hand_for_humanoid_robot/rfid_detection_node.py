#!/usr/bin/env python3

import time
import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool


class RfidDetectionNode(Node):
    def __init__(self):
        super().__init__('rfid_detection_node')

        self.declare_parameter(
            'device_name',
            '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F42208A55157375037202020FF122F34-if00'
        )

        # Must match the merged OpenRB sketch USB.begin(57600).
        self.declare_parameter('baudrate', 57600)

        # For testing, keep running and allow multiple scans.
        self.declare_parameter('exit_after_detection', False)

        # If True, send "BRIDGE" before exiting so OpenRB is ready for Dynamixel mode.
        self.declare_parameter('switch_to_bridge_on_exit', True)

        # If True, print every serial line from OpenRB.
        # Keep False while debugging noisy output.
        self.declare_parameter('log_raw_serial', False)

        self.declare_parameter('tool_uid_map', [
            '04A2913B6C7180=Test Tool A',
            '04B12345678901=Test Tool B',
            '04777777777777=Test Tool C',
        ])

        self.device_name = str(self.get_parameter('device_name').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.exit_after_detection = bool(
            self.get_parameter('exit_after_detection').value
        )
        self.switch_to_bridge_on_exit = bool(
            self.get_parameter('switch_to_bridge_on_exit').value
        )
        self.log_raw_serial = bool(
            self.get_parameter('log_raw_serial').value
        )

        self.tool_uid_map = self.parse_tool_uid_map(
            list(self.get_parameter('tool_uid_map').value)
        )

        self.uid_pub = self.create_publisher(
            String,
            '/h4hr/rfid_uid',
            10
        )

        self.tool_pub = self.create_publisher(
            String,
            '/h4hr/detected_tool',
            10
        )

        self.complete_pub = self.create_publisher(
            Bool,
            '/h4hr/rfid_detection_complete',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/h4hr/rfid_status',
            10
        )

        self.serial_port = None
        self.last_uid = None

        self.open_serial_port()
        self.enter_rfid_mode()

        # Slower timer. We do not need to hammer the serial port.
        self.timer = self.create_timer(
            0.20,
            self.read_serial_once
        )

        self.publish_status('RFID detection node started.')
        self.publish_status(f'Listening on {self.device_name} at {self.baudrate} baud.')
        self.publish_status('Waiting for RFID sticker scan...')

    def parse_tool_uid_map(self, entries):
        tool_map = {}

        for entry in entries:
            if '=' not in entry:
                continue

            uid, tool_name = entry.split('=', 1)
            uid = uid.strip().upper()
            tool_name = tool_name.strip()

            if uid:
                tool_map[uid] = tool_name

        return tool_map

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def open_serial_port(self):
        try:
            self.serial_port = serial.Serial(
                self.device_name,
                self.baudrate,
                timeout=0.05
            )

            # Opening USB serial usually resets the OpenRB.
            time.sleep(2.0)

            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

        except serial.SerialException as error:
            self.get_logger().error(
                f'Could not open RFID serial port {self.device_name}: {error}'
            )
            raise

    def enter_rfid_mode(self):
        if self.serial_port is None:
            return

        self.publish_status('Requesting OpenRB RFID mode...')

        try:
            self.serial_port.write(b'RFID\n')
            self.serial_port.flush()
            time.sleep(0.5)

        except serial.SerialException as error:
            self.get_logger().error(f'Failed to request RFID mode: {error}')

    def enter_bridge_mode(self):
        if self.serial_port is None:
            return

        try:
            self.publish_status('Requesting OpenRB DXL bridge mode...')
            self.serial_port.write(b'BRIDGE\n')
            self.serial_port.flush()
            time.sleep(0.2)

        except serial.SerialException as error:
            self.get_logger().error(f'Failed to request bridge mode: {error}')

    def read_serial_once(self):
        if self.serial_port is None:
            return

        try:
            line = self.serial_port.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()

        except serial.SerialException as error:
            self.get_logger().error(f'RFID serial read error: {error}')
            return

        # No constant waiting messages.
        if not line:
            return

        if self.log_raw_serial:
            self.get_logger().info(f'RFID raw serial: {line}')

        if line.startswith('PN532_OPENRB_READY'):
            self.publish_status('OpenRB RFID firmware is ready.')
            return

        if line.startswith('Initializing PN532'):
            self.publish_status('Initializing PN532...')
            return

        if line.startswith('PN532_FOUND'):
            self.publish_status(line)
            return

        if line.startswith('READY_TO_SCAN'):
            self.publish_status('PN532 ready to scan RFID sticker.')
            return

        if line.startswith('ALREADY_IN_RFID_MODE'):
            self.publish_status('OpenRB is already in RFID mode.')
            return

        if line.startswith('SWITCHING_TO_DXL_BRIDGE'):
            self.publish_status('OpenRB switching to DXL bridge mode.')
            return

        if line.startswith('ERROR:'):
            self.publish_status(line)
            return

        if not line.startswith('UID:'):
            return

        uid = line.replace('UID:', '').strip().upper()

        if not uid:
            return

        self.handle_uid(uid)

    def handle_uid(self, uid):
        self.last_uid = uid

        uid_msg = String()
        uid_msg.data = uid
        self.uid_pub.publish(uid_msg)

        tool_name = self.tool_uid_map.get(
            uid,
            f'UNKNOWN_TOOL_UID_{uid}'
        )

        tool_msg = String()
        tool_msg.data = tool_name
        self.tool_pub.publish(tool_msg)

        complete_msg = Bool()
        complete_msg.data = True
        self.complete_pub.publish(complete_msg)

        self.publish_status('--------------------------------')
        self.publish_status(f'RFID UID detected: {uid}')
        self.publish_status(f'Detected tool: {tool_name}')
        self.publish_status('Scan another sticker, or press CTRL+C to stop.')
        self.publish_status('--------------------------------')

        if self.exit_after_detection:
            self.shutdown_node()

    def shutdown_node(self):
        self.publish_status('RFID node exiting and releasing serial port.')

        if self.switch_to_bridge_on_exit:
            self.enter_bridge_mode()

        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass

            self.serial_port = None

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = RfidDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_status('Keyboard interrupt. Closing RFID node.')

    if node.switch_to_bridge_on_exit:
        node.enter_bridge_mode()

    if node.serial_port is not None:
        try:
            node.serial_port.close()
        except serial.SerialException:
            pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()