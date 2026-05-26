#!/usr/bin/env python3

import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


class SimpleJointKeyboardNode(Node):
    def __init__(self):
        super().__init__('simple_joint_keyboard_node')

        self.device_name = '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F42208A55157375037202020FF122F34-if00'
        self.baudrate = 57600

        # Motor ID 1 = Disk 1
        # Motor ID 2 = Disk 2
        # Motor ID 3 = Disk 3
        # Motor ID 4 = Disk 4
        self.motor_ids = [1, 2, 3, 4]

        # Your already-found zero positions.
        # Change these to your final known zero/home values.
        self.home_positions = [2048, 4096, -1024, 2048]

        self.step_deg = 0.2
        self.counts_per_degree = 4096.0 / 360.0

        self.ADDR_OPERATING_MODE = 11
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_PWM = 124
        self.ADDR_PRESENT_CURRENT = 126

        self.TORQUE_DISABLE = 0
        self.TORQUE_ENABLE = 1
        self.EXTENDED_POSITION_CONTROL_MODE = 4

        self.port = PortHandler(self.device_name)
        self.packet = PacketHandler(2.0)

        # Coupling matrix:
        # Roll  = -1.56323325 * D1
        # Pitch =  1.01857984 * D2
        # Yaw   = -0.830634273 * D2 + 0.608862987 * D3 + 0.608862987 * D4
        # Grip  = -1.21772597 * D3 + 1.21772597 * D4
        self.ROLL_D1 = -1.56323325
        self.PITCH_D2 = 1.01857984
        self.YAW_D2 = -0.830634273
        self.YAW_D3 = 0.608862987
        self.GRIP_D4 = 1.21772597

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.grip = 0.0

        self.connect()
        self.print_controls()

    def connect(self):
        if not self.port.openPort():
            raise RuntimeError('Failed to open Dynamixel port.')

        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError('Failed to set baudrate.')

        for motor_id in self.motor_ids:
            self.ping(motor_id)
            self.set_extended_position_mode(motor_id)

        self.get_logger().warn(f'Using fixed zero/home positions: {self.home_positions}')

    def ping(self, motor_id):
        model, result, error = self.packet.ping(self.port, motor_id)

        if result == COMM_SUCCESS and error == 0:
            self.get_logger().info(f'Motor {motor_id} found. Model: {model}')
        else:
            self.get_logger().error(f'Motor {motor_id} ping failed.')

    def write1(self, motor_id, address, value):
        result, error = self.packet.write1ByteTxRx(
            self.port,
            motor_id,
            address,
            int(value)
        )

        if result != COMM_SUCCESS:
            self.get_logger().error(self.packet.getTxRxResult(result))

        if error != 0:
            self.get_logger().error(self.packet.getRxPacketError(error))

    def write4(self, motor_id, address, value):
        result, error = self.packet.write4ByteTxRx(
            self.port,
            motor_id,
            address,
            int(value) & 0xFFFFFFFF
        )

        if result != COMM_SUCCESS:
            self.get_logger().error(self.packet.getTxRxResult(result))

        if error != 0:
            self.get_logger().error(self.packet.getRxPacketError(error))

    def unsigned_to_signed_16(self, value):
        value = int(value)

        if value > 0x7FFF:
            value -= 0x10000

        return value


def read2(self, motor_id, address):
    value, result, error = self.packet.read2ByteTxRx(
        self.port,
        motor_id,
        address
    )

    if result != COMM_SUCCESS:
        self.get_logger().error(self.packet.getTxRxResult(result))
        return None

    if error != 0:
        self.get_logger().error(self.packet.getRxPacketError(error))
        return None

    return self.unsigned_to_signed_16(value)


def print_currents(self):
    currents = []
    pwms = []

    for motor_id in self.motor_ids:
        raw_current = self.read2(motor_id, self.ADDR_PRESENT_CURRENT)
        raw_pwm = self.read2(motor_id, self.ADDR_PRESENT_PWM)

        if raw_current is None:
            raw_current = 0

        if raw_pwm is None:
            raw_pwm = 0

        current_A = raw_current / 1000.0

        currents.append(current_A)
        pwms.append(raw_pwm)

    self.get_logger().warn(
        f"Current A row: "
        f"[{currents[0]:.3f}, {currents[1]:.3f}, "
        f"{currents[2]:.3f}, {currents[3]:.3f}]"
    )

    self.get_logger().warn(
        f"Present PWM row: "
        f"[{pwms[0]}, {pwms[1]}, {pwms[2]}, {pwms[3]}]"
    )

    def set_extended_position_mode(self, motor_id):
        self.write1(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self.write1(motor_id, self.ADDR_OPERATING_MODE, self.EXTENDED_POSITION_CONTROL_MODE)
        self.write1(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

    def joints_to_disks(self):
        d1 = self.roll / self.ROLL_D1
        d2 = self.pitch / self.PITCH_D2

        d3_plus_d4 = (self.yaw - self.YAW_D2 * d2) / self.YAW_D3
        d4_minus_d3 = self.grip / self.GRIP_D4

        d3 = 0.5 * (d3_plus_d4 - d4_minus_d3)
        d4 = 0.5 * (d3_plus_d4 + d4_minus_d3)

        return [d1, d2, d3, d4]

    def command_joints(self):
        disks = self.joints_to_disks()

        self.get_logger().info(
            f'Joints deg: R={self.roll:.1f}, P={self.pitch:.1f}, '
            f'Y={self.yaw:.1f}, G={self.grip:.1f}'
        )

        self.get_logger().info(
            f'Disks deg: D1={disks[0]:.1f}, D2={disks[1]:.1f}, '
            f'D3={disks[2]:.1f}, D4={disks[3]:.1f}'
        )

        for i, motor_id in enumerate(self.motor_ids):
            goal = int(self.home_positions[i] + disks[i] * self.counts_per_degree)
            self.write4(motor_id, self.ADDR_GOAL_POSITION, goal)
            self.print_currents()

    def go_zero(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.grip = 0.0
        self.command_joints()

    def get_key(self):
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
            return key

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def run(self):
        while rclpy.ok():
            key = self.get_key()

            if key == '`':
                break

            elif key == '0':
                self.go_zero()

            elif key == '1':
                self.torque_off()

            elif key == '+':
                self.step_deg += 0.2
                self.get_logger().info(f'Step = {self.step_deg:.1f} deg')

            elif key == '-':
                self.step_deg = max(0.1, self.step_deg - 1.0)
                self.get_logger().info(f'Step = {self.step_deg:.1f} deg')

            elif key == 'q':
                self.roll += self.step_deg
                self.command_joints()

            elif key == 'e':
                self.roll -= self.step_deg
                self.command_joints()

            elif key == 'u':
                self.pitch += self.step_deg
                self.command_joints()

            elif key == 'o':
                self.pitch -= self.step_deg
                self.command_joints()

            elif key == 'a':
                self.yaw += self.step_deg
                self.command_joints()

            elif key == 'd':
                self.yaw -= self.step_deg
                self.command_joints()

            elif key == 'j':
                self.grip += self.step_deg
                self.command_joints()

            elif key == 'l':
                self.grip -= self.step_deg
                self.command_joints()
            elif key == 'p':
                self.print_currents()

    def torque_off(self):
        self.get_logger().warn('TORQUE OFF')

        for motor_id in self.motor_ids:
            self.write1(motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)

    def shutdown(self):
        self.torque_off()
        self.port.closePort()

    def print_controls(self):
        print()
        print('Simple Joint Keyboard Node')
        print('--------------------------')
        print('Uses fixed known zero positions.')
        print('No limits are active.')
        print()
        print('0  = go to fixed zero/home')
        print('q/e = roll +/-')
        print('u/o = pitch +/-')
        print('a/d = yaw +/-')
        print('j/l = grip +/-')
        print('+/- = change step size')
        print('1  = torque off')
        print('`  = quit')
        print()
        print(f'Fixed zero/home positions: {self.home_positions}')
        print(f'Step size: {self.step_deg:.1f} deg')
        print()


def main(args=None):
    rclpy.init(args=args)

    node = SimpleJointKeyboardNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()