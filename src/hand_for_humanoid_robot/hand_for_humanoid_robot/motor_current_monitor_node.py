#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


class MotorCurrentMonitorNode(Node):
    def __init__(self):
        super().__init__('motor_current_monitor_node')

        # -----------------------------
        # Basic settings
        # -----------------------------
        self.device_name = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F42208A55157375037202020FF122F34-if00'
        self.baudrate = 57600
        self.motor_ids = [1, 2, 3, 4]

        # How often to read and publish current
        self.publish_rate_hz = 10.0

        # -----------------------------
        # Dynamixel settings
        # -----------------------------
        self.protocol_version = 2.0

        # XC330 Present Current address
        # 2-byte signed value, approximately 1 mA per unit
        self.ADDR_PRESENT_CURRENT = 126

        self.port = PortHandler(self.device_name)
        self.packet = PacketHandler(self.protocol_version)

        # Publishes numerical array:
        # data: [motor1_A, motor2_A, motor3_A, motor4_A]
        self.current_pub = self.create_publisher(
            Float32MultiArray,
            '/h4hr/motor_currents_amps',
            10
        )

        # Publishes readable row vector string:
        # "[0.021, 0.034, -0.018, 0.026] A"
        self.current_row_pub = self.create_publisher(
            String,
            '/h4hr/motor_currents_amps_row',
            10
        )

        self.connect()

        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.publish_currents)

        self.get_logger().info('Motor current monitor started.')
        self.get_logger().info('Publishing:')
        self.get_logger().info('  /h4hr/motor_currents_amps')
        self.get_logger().info('  /h4hr/motor_currents_amps_row')

    def connect(self):
        if not self.port.openPort():
            raise RuntimeError(f'Failed to open port: {self.device_name}')

        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError(f'Failed to set baudrate: {self.baudrate}')

        self.get_logger().info(f'Opened port: {self.device_name}')

        for motor_id in self.motor_ids:
            model, result, error = self.packet.ping(self.port, motor_id)

            if result == COMM_SUCCESS and error == 0:
                self.get_logger().info(f'Motor {motor_id} found. Model: {model}')
            else:
                self.get_logger().error(f'Motor {motor_id} ping failed.')

    def unsigned_to_signed_16(self, value):
        value = int(value)

        if value > 0x7FFF:
            value -= 0x10000

        return value

    def read_current_amps(self, motor_id):
        raw, result, error = self.packet.read2ByteTxRx(
            self.port,
            motor_id,
            self.ADDR_PRESENT_CURRENT
        )

        if result != COMM_SUCCESS:
            self.get_logger().error(
                f'Motor {motor_id} current read failed: {self.packet.getTxRxResult(result)}'
            )
            return 0.0

        if error != 0:
            self.get_logger().error(
                f'Motor {motor_id} current read error: {self.packet.getRxPacketError(error)}'
            )
            return 0.0

        current_mA = self.unsigned_to_signed_16(raw)

        # XC330 current unit is approximately 1 mA
        current_A = current_mA / 1000.0

        return current_A

    def publish_currents(self):
        currents = []

        for motor_id in self.motor_ids:
            current_A = self.read_current_amps(motor_id)
            currents.append(current_A)

        array_msg = Float32MultiArray()
        array_msg.data = currents
        self.current_pub.publish(array_msg)

        row_msg = String()
        row_msg.data = (
            f"[{currents[0]:.3f}, "
            f"{currents[1]:.3f}, "
            f"{currents[2]:.3f}, "
            f"{currents[3]:.3f}] A"
        )
        self.current_row_pub.publish(row_msg)

        self.get_logger().info(
            f"Current A: "
            f"M1={currents[0]:.3f}, "
            f"M2={currents[1]:.3f}, "
            f"M3={currents[2]:.3f}, "
            f"M4={currents[3]:.3f}"
        )

    def shutdown(self):
        self.port.closePort()
        self.get_logger().info('Closed Dynamixel port.')


def main(args=None):
    rclpy.init(args=args)

    node = MotorCurrentMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
