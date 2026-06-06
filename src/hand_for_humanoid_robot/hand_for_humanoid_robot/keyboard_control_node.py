#!/usr/bin/env python3

import sys
import termios
import tty
import time
import threading

import rclpy
from rclpy.node import Node

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        self.declare_parameter(
            'device_name',
            '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F42208A55157375037202020FF122F34-if00'
        )
        self.declare_parameter('baudrate', 57600)

        self.declare_parameter('motor_ids', [1, 2, 3, 4])
        self.declare_parameter('home_positions', [2048, 2048, 1024, 1024])

        self.declare_parameter('joint_step_deg', 1.0)
        self.declare_parameter('diagonal_roll_step_deg', 1.0)

        # Close command goes slightly farther now.
        # Since gripper direction is inverted, close_gripper() uses this value.
        self.declare_parameter('grip_open_deg', 38.0)
        self.declare_parameter('grip_closed_deg', -15.0)

        self.declare_parameter('joint_speed_deg_per_sec', 45.0)
        self.declare_parameter('motion_loop_sleep', 0.01)
        self.declare_parameter('profile_velocity', 180)

        self.declare_parameter('arrival_tolerance_counts', 70)
        self.declare_parameter('arrival_timeout_sec', 15.0)

        self.declare_parameter('verbose_motion_logging', False)
        self.declare_parameter('torque_off_on_shutdown', True)

        self.device_name = self.get_parameter('device_name').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.motor_ids = list(self.get_parameter('motor_ids').value)
        self.home_positions = list(self.get_parameter('home_positions').value)

        self.joint_step_deg = float(self.get_parameter('joint_step_deg').value)
        self.diagonal_roll_step_deg = float(
            self.get_parameter('diagonal_roll_step_deg').value
        )

        self.grip_open_deg = float(self.get_parameter('grip_open_deg').value)
        self.grip_closed_deg = float(self.get_parameter('grip_closed_deg').value)

        self.joint_speed_deg_per_sec = float(
            self.get_parameter('joint_speed_deg_per_sec').value
        )
        self.motion_loop_sleep = float(self.get_parameter('motion_loop_sleep').value)
        self.profile_velocity = int(self.get_parameter('profile_velocity').value)
        self.arrival_tolerance_counts = int(self.get_parameter('arrival_tolerance_counts').value)
        self.arrival_timeout_sec = float(self.get_parameter('arrival_timeout_sec').value)

        self.verbose_motion_logging = bool(
            self.get_parameter('verbose_motion_logging').value
        )
        self.torque_off_on_shutdown = bool(
            self.get_parameter('torque_off_on_shutdown').value
        )

        self.protocol_version = 2.0

        self.ADDR_OPERATING_MODE = 11
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_PROFILE_VELOCITY = 112
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132

        self.TORQUE_DISABLE = 0
        self.TORQUE_ENABLE = 1
        self.EXTENDED_POSITION_CONTROL_MODE = 4

        self.COUNTS_PER_DEGREE = 4096.0 / 360.0

        # Motor direction correction.
        # Motors 3 and 4 are reversed because their physical clockwise/counterclockwise
        # directions are opposite of what the coupling matrix expects.
        self.motor_direction = {
            1: 1.0,
            2: 1.0,
            3: -1.0,
            4: -1.0,
        }

        # dVRK coupling matrix.
        self.ROLL_D1 = -1.56323325
        self.PITCH_D2 = 1.01857984
        self.YAW_D2 = -0.830634273
        self.YAW_D3 = 0.608862987
        self.YAW_D4 = 0.608862987
        self.GRIP_D3 = -1.21772597
        self.GRIP_D4 = 1.21772597

        self.joint_limits_deg = {
            'roll': (-259.0, 259.0),
            'pitch': (-79.0, 79.0),
            'yaw': (-79.0, 79.0),
            'grip': (-15.0, 38.0),
        }

        self.disk_angle_limits_deg = {
            1: (-166.0, 166.0),
            2: (-78.5, 78.5),
            3: (-150.0, 150.0),
            4: (-153.0, 153.0),
        }

        self.motor_position_limits = {}

        for dxl_id, home_position in zip(self.motor_ids, self.home_positions):
            min_deg, max_deg = self.disk_angle_limits_deg[dxl_id]

            min_count_raw = self.disk_angle_to_motor_position(dxl_id, min_deg)
            max_count_raw = self.disk_angle_to_motor_position(dxl_id, max_deg)

            self.motor_position_limits[dxl_id] = (
                min(min_count_raw, max_count_raw),
                max(min_count_raw, max_count_raw),
            )

        self.joint_targets_deg = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'grip': 0.0,
        }

        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        self.goal_positions = {}
        self.dxl_lock = threading.Lock()

        self.connect_to_motors()
        self.sync_joint_targets_to_present_position()
        self.print_instructions()

    def log_motion(self, text):
        if self.verbose_motion_logging:
            self.get_logger().info(text)

    def connect_to_motors(self):
        if len(self.motor_ids) != len(self.home_positions):
            raise RuntimeError('motor_ids and home_positions must be same length.')

        if not self.port_handler.openPort():
            raise RuntimeError(f'Failed to open port: {self.device_name}')

        self.get_logger().info(f'Opened port: {self.device_name}')

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f'Failed to set baudrate: {self.baudrate}')

        self.get_logger().info(f'Set baudrate: {self.baudrate}')
        self.get_logger().info(f'Motor position limits: {self.motor_position_limits}')

        for dxl_id in self.motor_ids:
            self.ping_motor(dxl_id)
            self.goal_positions[dxl_id] = self.get_home_position(dxl_id)

        self.set_all_extended_position_mode()

        for dxl_id in self.motor_ids:
            self.sync_goal_to_present_position(dxl_id)

        self.hold_current_positions()

    def check_result(self, comm_result, dxl_error, action):
        if comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f'{action} failed: {self.packet_handler.getTxRxResult(comm_result)}'
            )
            return False

        if dxl_error != 0:
            self.get_logger().error(
                f'{action} error: {self.packet_handler.getRxPacketError(dxl_error)}'
            )
            return False

        return True

    def ping_motor(self, dxl_id):
        with self.dxl_lock:
            model_number, comm_result, dxl_error = self.packet_handler.ping(
                self.port_handler,
                dxl_id
            )

        if self.check_result(comm_result, dxl_error, f'Ping motor {dxl_id}'):
            self.get_logger().info(f'Motor {dxl_id} found. Model: {model_number}')
            return True

        self.get_logger().warn(f'Motor {dxl_id} not found.')
        return False

    def write_1_byte(self, dxl_id, address, value, action):
        with self.dxl_lock:
            comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                address,
                int(value)
            )

        return self.check_result(comm_result, dxl_error, action)

    def write_4_byte(self, dxl_id, address, value, action):
        with self.dxl_lock:
            comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler,
                dxl_id,
                address,
                int(value)
            )

        return self.check_result(comm_result, dxl_error, action)

    def read_4_byte(self, dxl_id, address, action):
        with self.dxl_lock:
            value, comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                self.port_handler,
                dxl_id,
                address
            )

        if self.check_result(comm_result, dxl_error, action):
            return value

        return None

    def signed_to_unsigned_32(self, value):
        return int(value) & 0xFFFFFFFF

    def unsigned_to_signed_32(self, value):
        value = int(value)

        if value > 0x7FFFFFFF:
            value -= 0x100000000

        return value

    def read_present_position(self, dxl_id):
        present = self.read_4_byte(
            dxl_id,
            self.ADDR_PRESENT_POSITION,
            f'Read present position motor {dxl_id}'
        )

        if present is None:
            return None

        return self.unsigned_to_signed_32(present)

    def sync_goal_to_present_position(self, dxl_id):
        present = self.read_present_position(dxl_id)

        if present is None:
            return False

        self.goal_positions[dxl_id] = present
        return True

    def get_home_position(self, dxl_id):
        if dxl_id not in self.motor_ids:
            return 0

        index = self.motor_ids.index(dxl_id)
        return int(self.home_positions[index])

    def get_motor_direction(self, dxl_id):
        return float(self.motor_direction.get(dxl_id, 1.0))

    def motor_position_to_disk_angle(self, dxl_id, position):
        home = self.get_home_position(dxl_id)
        direction = self.get_motor_direction(dxl_id)
        return float(position - home) / (direction * self.COUNTS_PER_DEGREE)

    def disk_angle_to_motor_position(self, dxl_id, disk_angle_deg):
        home = self.get_home_position(dxl_id)
        direction = self.get_motor_direction(dxl_id)
        return int(home + direction * disk_angle_deg * self.COUNTS_PER_DEGREE)

    def is_position_inside_bounds(self, dxl_id, position):
        if dxl_id not in self.motor_position_limits:
            return False

        min_position, max_position = self.motor_position_limits[dxl_id]
        return min_position <= int(position) <= max_position

    def clamp_joint(self, joint_name, angle_deg):
        if joint_name not in self.joint_limits_deg:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return None

        min_deg, max_deg = self.joint_limits_deg[joint_name]

        if angle_deg < min_deg:
            self.get_logger().warn(
                f'{joint_name} command {angle_deg:.1f} deg below limit {min_deg:.1f}. '
                f'Clamping.'
            )
            return min_deg

        if angle_deg > max_deg:
            self.get_logger().warn(
                f'{joint_name} command {angle_deg:.1f} deg above limit {max_deg:.1f}. '
                f'Clamping.'
            )
            return max_deg

        return float(angle_deg)

    def clamp_position(self, dxl_id, position):
        if dxl_id not in self.motor_position_limits:
            self.get_logger().error(f'No position limits defined for motor {dxl_id}.')
            return None

        min_position, max_position = self.motor_position_limits[dxl_id]

        if position < min_position:
            self.get_logger().warn(
                f'Motor {dxl_id} command {position} below disk safe limit {min_position}. '
                f'Clamping to {min_position}.'
            )
            return min_position

        if position > max_position:
            self.get_logger().warn(
                f'Motor {dxl_id} command {position} above disk safe limit {max_position}. '
                f'Clamping to {max_position}.'
            )
            return max_position

        return int(position)

    def command_position(self, dxl_id, position, log=True):
        safe_position = self.clamp_position(dxl_id, position)

        if safe_position is None:
            return False

        self.goal_positions[dxl_id] = safe_position

        ok = self.write_4_byte(
            dxl_id,
            self.ADDR_GOAL_POSITION,
            self.signed_to_unsigned_32(safe_position),
            f'Set goal position motor {dxl_id}'
        )

        if ok and log and self.verbose_motion_logging:
            disk_angle = self.motor_position_to_disk_angle(dxl_id, safe_position)
            self.get_logger().info(
                f'Motor {dxl_id} / Disk {dxl_id} goal: '
                f'{safe_position} counts, {disk_angle:.1f} deg from home'
            )

        return ok

    def joints_to_disks(self, joints_deg):
        roll = float(joints_deg['roll'])
        pitch = float(joints_deg['pitch'])
        yaw = float(joints_deg['yaw'])
        grip = float(joints_deg['grip'])

        d1 = roll / self.ROLL_D1
        d2 = pitch / self.PITCH_D2

        d3_plus_d4 = (yaw - self.YAW_D2 * d2) / self.YAW_D3
        d4_minus_d3 = grip / self.GRIP_D4

        d3 = 0.5 * (d3_plus_d4 - d4_minus_d3)
        d4 = 0.5 * (d3_plus_d4 + d4_minus_d3)

        return {
            1: d1,
            2: d2,
            3: d3,
            4: d4,
        }

    def disks_to_joints(self, disks_deg):
        d1 = float(disks_deg[1])
        d2 = float(disks_deg[2])
        d3 = float(disks_deg[3])
        d4 = float(disks_deg[4])

        return {
            'roll': self.ROLL_D1 * d1,
            'pitch': self.PITCH_D2 * d2,
            'yaw': self.YAW_D2 * d2 + self.YAW_D3 * d3 + self.YAW_D4 * d4,
            'grip': self.GRIP_D3 * d3 + self.GRIP_D4 * d4,
        }

    def validate_disk_angles(self, disk_angles_deg):
        for dxl_id, disk_angle in disk_angles_deg.items():
            if dxl_id not in self.disk_angle_limits_deg:
                self.get_logger().error(f'No disk angle limit for disk/motor {dxl_id}.')
                return False

            min_deg, max_deg = self.disk_angle_limits_deg[dxl_id]

            if disk_angle < min_deg or disk_angle > max_deg:
                self.get_logger().error(
                    f'Joint command refused: Disk {dxl_id} would move to '
                    f'{disk_angle:.1f} deg, outside safe range '
                    f'[{min_deg:.1f}, {max_deg:.1f}] deg.'
                )
                return False

        return True

    def joint_targets_to_motor_positions(self, joints_deg):
        disk_angles = self.joints_to_disks(joints_deg)

        if not self.validate_disk_angles(disk_angles):
            return None

        motor_positions = {}

        for dxl_id, disk_angle in disk_angles.items():
            motor_positions[dxl_id] = self.disk_angle_to_motor_position(
                dxl_id,
                disk_angle
            )

        return motor_positions

    def present_disk_angles(self):
        disks = {}

        for dxl_id in self.motor_ids:
            present = self.read_present_position(dxl_id)

            if present is None:
                disks[dxl_id] = 0.0
                continue

            disks[dxl_id] = self.motor_position_to_disk_angle(dxl_id, present)

        return disks

    def sync_joint_targets_to_present_position(self):
        disks = self.present_disk_angles()
        joints = self.disks_to_joints(disks)

        for name in self.joint_targets_deg:
            joints[name] = self.clamp_joint(name, joints[name])

        self.joint_targets_deg = joints

        self.get_logger().info(
            'Synced keyboard targets to current motor positions: '
            f"R={joints['roll']:.1f}, "
            f"P={joints['pitch']:.1f}, "
            f"Y={joints['yaw']:.1f}, "
            f"G={joints['grip']:.1f}"
        )

    def print_joint_and_disk_targets(self, joints_deg):
        disk_angles = self.joints_to_disks(joints_deg)

        self.get_logger().info(
            'Joint targets: '
            f"Roll={joints_deg['roll']:.1f}, "
            f"Pitch={joints_deg['pitch']:.1f}, "
            f"Yaw={joints_deg['yaw']:.1f}, "
            f"Grip={joints_deg['grip']:.1f} deg"
        )

        self.get_logger().info(
            'Disk targets from coupling matrix: '
            f"D1={disk_angles[1]:.1f}, "
            f"D2={disk_angles[2]:.1f}, "
            f"D3={disk_angles[3]:.1f}, "
            f"D4={disk_angles[4]:.1f} deg"
        )

    def set_motor_extended_position_mode(self, dxl_id):
        self.write_1_byte(
            dxl_id,
            self.ADDR_TORQUE_ENABLE,
            self.TORQUE_DISABLE,
            f'Disable torque motor {dxl_id}'
        )

        self.write_1_byte(
            dxl_id,
            self.ADDR_OPERATING_MODE,
            self.EXTENDED_POSITION_CONTROL_MODE,
            f'Set extended position mode motor {dxl_id}'
        )

        self.write_4_byte(
            dxl_id,
            self.ADDR_PROFILE_VELOCITY,
            self.profile_velocity,
            f'Set profile velocity motor {dxl_id}'
        )

        self.write_1_byte(
            dxl_id,
            self.ADDR_TORQUE_ENABLE,
            self.TORQUE_ENABLE,
            f'Enable torque motor {dxl_id}'
        )

    def set_all_extended_position_mode(self):
        for dxl_id in self.motor_ids:
            self.set_motor_extended_position_mode(dxl_id)

        self.get_logger().info('Set all motors to EXTENDED POSITION control mode.')

    def run_independent_motor_sequences(self, sequences, speed_deg_per_sec):
        states = {}
        speed_deg_per_sec = abs(float(speed_deg_per_sec))

        if speed_deg_per_sec <= 0.0:
            self.get_logger().error('Sequence speed must be greater than 0 deg/sec.')
            return False

        speed_counts_per_sec = speed_deg_per_sec * self.COUNTS_PER_DEGREE

        for dxl_id, waypoints in sequences.items():
            if dxl_id not in self.motor_ids:
                self.get_logger().warn(f'Motor {dxl_id} ignored: not in motor_ids.')
                continue

            if not waypoints:
                self.get_logger().warn(f'Motor {dxl_id} ignored: no waypoints.')
                continue

            valid_waypoints = []

            for target_position in waypoints:
                target_position = int(target_position)

                if not self.is_position_inside_bounds(dxl_id, target_position):
                    min_position, max_position = self.motor_position_limits[dxl_id]
                    self.get_logger().error(
                        f'Motor {dxl_id} sequence refused: waypoint {target_position} '
                        f'outside bounds [{min_position}, {max_position}].'
                    )
                    valid_waypoints = []
                    break

                valid_waypoints.append(target_position)

            if not valid_waypoints:
                continue

            present = self.read_present_position(dxl_id)

            if present is None:
                self.get_logger().error(
                    f'Motor {dxl_id} sequence refused: could not read present position.'
                )
                continue

            if not self.is_position_inside_bounds(dxl_id, present):
                min_position, max_position = self.motor_position_limits[dxl_id]
                self.get_logger().error(
                    f'Motor {dxl_id} sequence refused: present {present} '
                    f'outside bounds [{min_position}, {max_position}].'
                )
                continue

            self.goal_positions[dxl_id] = present

            states[dxl_id] = {
                'waypoints': valid_waypoints,
                'index': 0,
                'last_update_time': time.time(),
                'waypoint_start_time': time.time(),
                'done': False,
            }

        if not states:
            self.get_logger().error('No valid motor sequences to run.')
            return False

        while rclpy.ok():
            all_done = True
            now = time.time()

            for dxl_id, state in states.items():
                if state['done']:
                    continue

                all_done = False

                waypoints = state['waypoints']
                waypoint_index = state['index']
                target_position = waypoints[waypoint_index]

                present = self.read_present_position(dxl_id)

                if present is None:
                    self.get_logger().error(
                        f'Motor {dxl_id} sequence stopped: could not read present position.'
                    )
                    state['done'] = True
                    continue

                present_error = target_position - present

                if abs(present_error) <= self.arrival_tolerance_counts:
                    self.command_position(dxl_id, target_position, log=False)

                    state['index'] += 1

                    if state['index'] >= len(waypoints):
                        state['done'] = True
                        continue

                    state['waypoint_start_time'] = now
                    state['last_update_time'] = now
                    continue

                elapsed_at_waypoint = now - state['waypoint_start_time']

                if elapsed_at_waypoint > self.arrival_timeout_sec:
                    self.get_logger().error(
                        f'Motor {dxl_id} sequence timeout at waypoint '
                        f'{waypoint_index + 1}/{len(waypoints)}. '
                        f'Target {target_position}, present {present}, '
                        f'error {present_error} counts, '
                        f'disk angle {self.motor_position_to_disk_angle(dxl_id, present):.1f} deg.'
                    )
                    state['done'] = True
                    continue

                dt = now - state['last_update_time']
                state['last_update_time'] = now

                step_counts = max(1, int(speed_counts_per_sec * dt))

                current_command = int(self.goal_positions.get(dxl_id, present))
                command_error = target_position - current_command

                if abs(command_error) <= step_counts:
                    next_command = target_position
                elif command_error > 0:
                    next_command = current_command + step_counts
                else:
                    next_command = current_command - step_counts

                if not self.is_position_inside_bounds(dxl_id, next_command):
                    self.get_logger().error(
                        f'Motor {dxl_id} sequence stopped: next command {next_command} '
                        f'would leave bounds.'
                    )
                    state['done'] = True
                    continue

                self.command_position(dxl_id, next_command, log=False)

            if all_done:
                return True

            time.sleep(self.motion_loop_sleep)

        return False

    def move_to_joint_targets(self, joints_deg, speed_deg_per_sec):
        motor_positions = self.joint_targets_to_motor_positions(joints_deg)

        if motor_positions is None:
            self.get_logger().error('Joint command refused.')
            return False

        sequences = {}

        for dxl_id, position in motor_positions.items():
            sequences[dxl_id] = [position]

        if self.verbose_motion_logging:
            self.print_joint_and_disk_targets(joints_deg)

        return self.run_independent_motor_sequences(sequences, speed_deg_per_sec)

    def apply_joint_delta(self, roll_delta, pitch_delta, yaw_delta, grip_delta=0.0):
        new_targets = dict(self.joint_targets_deg)

        deltas = {
            'roll': roll_delta,
            'pitch': pitch_delta,
            'yaw': yaw_delta,
            'grip': grip_delta,
        }

        for joint_name, delta in deltas.items():
            new_angle = new_targets[joint_name] + delta
            new_angle = self.clamp_joint(joint_name, new_angle)

            if new_angle is None:
                return

            new_targets[joint_name] = new_angle

        ok = self.move_to_joint_targets(new_targets, self.joint_speed_deg_per_sec)

        if ok:
            self.joint_targets_deg = new_targets

    def set_joint_target(self, joint_name, angle_deg):
        if joint_name not in self.joint_targets_deg:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return

        new_targets = dict(self.joint_targets_deg)
        angle_deg = self.clamp_joint(joint_name, angle_deg)

        if angle_deg is None:
            return

        new_targets[joint_name] = angle_deg

        ok = self.move_to_joint_targets(new_targets, self.joint_speed_deg_per_sec)

        if ok:
            self.joint_targets_deg = new_targets

    def zero_all_joints(self):
        new_targets = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'grip': 0.0,
        }

        ok = self.move_to_joint_targets(new_targets, self.joint_speed_deg_per_sec)

        if ok:
            self.joint_targets_deg = new_targets

    def open_gripper(self):
        # Inverted grip direction.
        self.set_joint_target('grip', self.grip_closed_deg)

    def close_gripper(self):
        # Inverted grip direction.
        # Close now goes slightly farther: 36 deg.
        self.set_joint_target('grip', self.grip_open_deg)

    def toggle_gripper(self):
        current_grip = self.joint_targets_deg['grip']
        halfway = 0.5 * (self.grip_open_deg + self.grip_closed_deg)

        if current_grip < halfway:
            self.close_gripper()
        else:
            self.open_gripper()

    def hold_current_positions(self):
        self.get_logger().info('Holding current motor positions.')

        for dxl_id in self.motor_ids:
            present = self.read_present_position(dxl_id)

            if present is None:
                continue

            if not self.is_position_inside_bounds(dxl_id, present):
                self.get_logger().warn(
                    f'Motor {dxl_id} present position {present} is outside bounds. '
                    f'Not commanding hold.'
                )
                continue

            self.command_position(dxl_id, present, log=False)

    def emergency_torque_off(self):
        self.get_logger().warn('EMERGENCY STOP: disabling torque on all motors.')

        for dxl_id in self.motor_ids:
            self.write_1_byte(
                dxl_id,
                self.ADDR_TORQUE_ENABLE,
                self.TORQUE_DISABLE,
                f'Emergency torque off motor {dxl_id}'
            )

    def decrease_step(self):
        self.joint_step_deg -= 1.0

        if self.joint_step_deg < 1.0:
            self.joint_step_deg = 1.0

        self.get_logger().info(f'Joint step decreased to {self.joint_step_deg:.1f} deg')

    def increase_step(self):
        self.joint_step_deg += 1.0

        if self.joint_step_deg > 20.0:
            self.joint_step_deg = 20.0

        self.get_logger().info(f'Joint step increased to {self.joint_step_deg:.1f} deg')

    def print_instructions(self):
        print()
        print('H4HR Keyboard Control Node - NUMPAD TOOL CONTROL')
        print('------------------------------------------------')
        print('This node directly opens the OpenRB/Dynamixel port.')
        print('Do NOT run this at the same time as auto_zero_node, dynamixel_controller_node, or rfid_detection_node.')
        print()
        print('Make sure Num Lock is ON.')
        print()
        print('MOTOR DIRECTION FIX:')
        print('  Motor 3 direction inverted')
        print('  Motor 4 direction inverted')
        print()
        print('NUMPAD CONTROL:')
        print('  7 : forward + left + roll left')
        print('  8 : forward')
        print('  9 : forward + right + roll right')
        print('  4 : left / yaw left')
        print('  5 : hold current position')
        print('  6 : right / yaw right')
        print('  1 : backward + left + roll left')
        print('  2 : backward')
        print('  3 : backward + right + roll right')
        print('  0 : return all joints to zero / home command')
        print('  . : toggle gripper open/close')
        print()
        print('OTHER:')
        print('  +  : increase joint step by 1 deg')
        print('  -  : decrease joint step by 1 deg')
        print('  e  : EMERGENCY TORQUE OFF')
        print('  `  : quit')
        print()
        print('CURRENT JOINT TARGETS:')
        for name, value in self.joint_targets_deg.items():
            print(f'  {name}: {value:.1f} deg')
        print()
        print('JOINT LIMITS:')
        for name, limits in self.joint_limits_deg.items():
            print(f'  {name}: {limits[0]:.1f} deg to {limits[1]:.1f} deg')
        print()
        print('DISK ANGLE SAFETY LIMITS:')
        for dxl_id in self.motor_ids:
            min_deg, max_deg = self.disk_angle_limits_deg[dxl_id]
            print(f'  Disk/Motor {dxl_id}: {min_deg:.1f} deg to {max_deg:.1f} deg')
        print()
        print(f'Joint step: {self.joint_step_deg:.1f} deg')
        print(f'Diagonal roll step: {self.diagonal_roll_step_deg:.1f} deg')
        print(f'Joint speed: {self.joint_speed_deg_per_sec:.1f} deg/sec')
        print(f'Verbose motion logging: {self.verbose_motion_logging}')
        print()
        print('IMPORTANT:')
        print('  Motor 3 and Motor 4 are direction-corrected in software.')
        print('  Left/right yaw signs are uninverted in this version.')
        print('  Gripper direction is inverted.')
        print('  open command -> grip 0 deg')
        print('  close command -> grip 36 deg')
        print('  Press e to disable torque immediately.')
        print()

    def get_key(self):
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
            return key

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def handle_key(self, key):
        step = self.joint_step_deg
        roll_step = self.diagonal_roll_step_deg

        if key == '`':
            return False

        # Forward/backward are still inverted.
        # Left/right yaw signs have been uninverted from the previous version.

        if key == '7':
            self.apply_joint_delta(+roll_step, -step, -step)
            return True

        if key == '8':
            self.apply_joint_delta(0.0, -step, 0.0)
            return True

        if key == '9':
            self.apply_joint_delta(-roll_step, -step, +step)
            return True

        if key == '4':
            self.apply_joint_delta(0.0, 0.0, -step)
            return True

        if key == '5':
            self.hold_current_positions()
            self.sync_joint_targets_to_present_position()
            return True

        if key == '6':
            self.apply_joint_delta(0.0, 0.0, +step)
            return True

        if key == '1':
            self.apply_joint_delta(+roll_step, +step, -step)
            return True

        if key == '2':
            self.apply_joint_delta(0.0, +step, 0.0)
            return True

        if key == '3':
            self.apply_joint_delta(-roll_step, +step, +step)
            return True

        if key == '0':
            self.zero_all_joints()
            return True

        if key == '.':
            self.toggle_gripper()
            return True

        if key == '+':
            self.increase_step()
            return True

        if key == '-':
            self.decrease_step()
            return True

        if key == 'e':
            self.emergency_torque_off()
            return True

        return True

    def run_keyboard_loop(self):
        running = True

        while rclpy.ok() and running:
            key = self.get_key()
            running = self.handle_key(key)

    def shutdown(self):
        if self.torque_off_on_shutdown:
            self.emergency_torque_off()
        else:
            self.get_logger().warn('Leaving motor torque enabled on shutdown.')

        self.port_handler.closePort()
        self.get_logger().info('Closed Dynamixel port.')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()

    try:
        node.run_keyboard_loop()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt.')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()