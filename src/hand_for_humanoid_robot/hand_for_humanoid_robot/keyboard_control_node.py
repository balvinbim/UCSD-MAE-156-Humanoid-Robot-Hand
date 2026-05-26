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

        # -----------------------------
        # ROS parameters
        # -----------------------------
        self.declare_parameter(
            'device_name',
            '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F42208A55157375037202020FF122F34-if00'
        )
        self.declare_parameter('baudrate', 57600)

        # Motor IDs correspond to Disk 1, Disk 2, Disk 3, Disk 4.
        self.declare_parameter('motor_ids', [1, 2, 3, 4])

        # Disk zero/home positions.
        # Motor 1 = Disk 1
        # Motor 2 = Disk 2
        # Motor 3 = Disk 3
        # Motor 4 = Disk 4
        self.declare_parameter('home_positions', [2048, 0, 3072, 2048])

        # Joint step size in degrees for keyboard jogging.
        self.declare_parameter('joint_step_deg', 1.0)

        # Open/close gripper target angles in degrees.
        self.declare_parameter('grip_open_deg', 30.0)
        self.declare_parameter('grip_closed_deg', 0.0)

        # Joint motion speed.
        self.declare_parameter('joint_speed_deg_per_sec', 35.0)

        # Sleep between command updates.
        self.declare_parameter('motion_loop_sleep', 0.01)

        # Dynamixel internal profile velocity.
        # Keep this high enough that the software speed command is the limiting factor.
        self.declare_parameter('profile_velocity', 80)

        # How close the physical motor position must be before a waypoint is considered reached.
        self.declare_parameter('arrival_tolerance_counts', 20)

        # Maximum time allowed per waypoint before that motor sequence is stopped.
        self.declare_parameter('arrival_timeout_sec', 45.0)

        self.device_name = self.get_parameter('device_name').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.motor_ids = list(self.get_parameter('motor_ids').value)
        self.home_positions = list(self.get_parameter('home_positions').value)

        self.joint_step_deg = float(self.get_parameter('joint_step_deg').value)
        self.grip_open_deg = float(self.get_parameter('grip_open_deg').value)
        self.grip_closed_deg = float(self.get_parameter('grip_closed_deg').value)

        self.joint_speed_deg_per_sec = float(
            self.get_parameter('joint_speed_deg_per_sec').value
        )
        self.motion_loop_sleep = float(self.get_parameter('motion_loop_sleep').value)
        self.profile_velocity = int(self.get_parameter('profile_velocity').value)
        self.arrival_tolerance_counts = int(self.get_parameter('arrival_tolerance_counts').value)
        self.arrival_timeout_sec = float(self.get_parameter('arrival_timeout_sec').value)

        # -----------------------------
        # Dynamixel settings
        # -----------------------------
        self.protocol_version = 2.0

        self.ADDR_OPERATING_MODE = 11
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_PROFILE_VELOCITY = 112
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132

        self.TORQUE_DISABLE = 0
        self.TORQUE_ENABLE = 1

        # Extended Position Control Mode allows position values outside 0-4095.
        self.EXTENDED_POSITION_CONTROL_MODE = 4

        # 4096 counts = 360 degrees.
        self.COUNTS_PER_DEGREE = 4096.0 / 360.0

        # -----------------------------
        # dVRK coupling matrix
        # -----------------------------
        # The uploaded dVRK matrix is:
        #
        # Roll  = -1.56323325 * Disk1
        # Pitch =  1.01857984 * Disk2
        # Yaw   = -0.830634273 * Disk2 + 0.608862987 * Disk3 + 0.608862987 * Disk4
        # Grip  = -1.21772597 * Disk3 + 1.21772597 * Disk4
        #
        # All angles are treated in degrees because the matrix is unitless.
        self.ROLL_D1 = -1.56323325
        self.PITCH_D2 = 1.01857984
        self.YAW_D2 = -0.830634273
        self.YAW_D3 = 0.608862987
        self.YAW_D4 = 0.608862987
        self.GRIP_D3 = -1.21772597
        self.GRIP_D4 = 1.21772597

        # Joint limits from the uploaded dVRK guide.
        self.joint_limits_deg = {
            'roll': (-259.0, 259.0),
            'pitch': (-79.0, 79.0),
            'yaw': (-80.0, 80.0),
            'grip': (0.0, 30.0),
        }

        # Extra disk-level safety limits.
        # These are limits on the physical disk/motor angle relative to home.
        # They prevent the coupling matrix from commanding extreme motor rotations.
        self.disk_angle_limits_deg = {
            1: (-166.0, 166.0),
            2: (-78.5, 78.5),
            3: (-150.0, 150.0),
            4: (-150.0, 150.0),
        }

        self.motor_position_limits = {}

        for dxl_id, home_position in zip(self.motor_ids, self.home_positions):
            min_deg, max_deg = self.disk_angle_limits_deg[dxl_id]
            min_count = int(home_position + min_deg * self.COUNTS_PER_DEGREE)
            max_count = int(home_position + max_deg * self.COUNTS_PER_DEGREE)
            self.motor_position_limits[dxl_id] = (min_count, max_count)

        # Current desired joint position in degrees.
        self.joint_targets_deg = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'grip': 0.0,
        }

        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        self.goal_positions = {}

        # Lock protects serial communication.
        self.dxl_lock = threading.Lock()

        self.connect_to_motors()
        self.print_instructions()

    # -----------------------------
    # Connection and communication
    # -----------------------------
    def connect_to_motors(self):
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

        self.move_to_joint_targets(self.joint_targets_deg, self.joint_speed_deg_per_sec)

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

    # -----------------------------
    # Signed / unsigned conversion
    # -----------------------------
    def signed_to_unsigned_32(self, value):
        return int(value) & 0xFFFFFFFF

    def unsigned_to_signed_32(self, value):
        value = int(value)

        if value > 0x7FFFFFFF:
            value -= 0x100000000

        return value

    # -----------------------------
    # Position helpers
    # -----------------------------
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

    def motor_position_to_disk_angle(self, dxl_id, position):
        home = self.get_home_position(dxl_id)
        return float(position - home) / self.COUNTS_PER_DEGREE

    def disk_angle_to_motor_position(self, dxl_id, disk_angle_deg):
        home = self.get_home_position(dxl_id)
        return int(home + disk_angle_deg * self.COUNTS_PER_DEGREE)

    def is_position_inside_bounds(self, dxl_id, position):
        if dxl_id not in self.motor_position_limits:
            return False

        min_position, max_position = self.motor_position_limits[dxl_id]
        return min_position <= position <= max_position

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

        if ok and log:
            disk_angle = self.motor_position_to_disk_angle(dxl_id, safe_position)
            self.get_logger().info(
                f'Motor {dxl_id} / Disk {dxl_id} goal: '
                f'{safe_position} counts, {disk_angle:.1f} deg from home'
            )

        return ok

    # -----------------------------
    # Coupling matrix helpers
    # -----------------------------
    def joints_to_disks(self, joints_deg):
        """
        Convert desired joint angles to disk angles.

        Matrix:
            Roll  = ROLL_D1 * D1
            Pitch = PITCH_D2 * D2
            Yaw   = YAW_D2 * D2 + YAW_D3 * D3 + YAW_D4 * D4
            Grip  = GRIP_D3 * D3 + GRIP_D4 * D4

        Since YAW_D3 == YAW_D4 and GRIP_D3 == -GRIP_D4, the D3/D4 solution is:

            D3 + D4 = (Yaw - YAW_D2 * D2) / YAW_D3
            D4 - D3 = Grip / GRIP_D4
        """

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

    # -----------------------------
    # Mode setup
    # -----------------------------
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

    # -----------------------------
    # Independent multi-motor sequence logic
    # -----------------------------
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

            waypoint_text = ', '.join(
                f'{position} ({self.motor_position_to_disk_angle(dxl_id, position):.1f} deg)'
                for position in valid_waypoints
            )

            self.get_logger().warn(
                f'Motor {dxl_id} / Disk {dxl_id} sequence armed from present {present} '
                f'({self.motor_position_to_disk_angle(dxl_id, present):.1f} deg) '
                f'to waypoints: {waypoint_text}'
            )

        if not states:
            self.get_logger().error('No valid motor sequences to run.')
            return False

        self.get_logger().warn(
            f'Running independent motor sequences for motors {list(states.keys())}. '
            f'Angular speed: {speed_deg_per_sec:.1f} deg/sec.'
        )

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

                    self.get_logger().info(
                        f'Motor {dxl_id} reached waypoint {waypoint_index + 1}/'
                        f'{len(waypoints)}: {target_position} '
                        f'({self.motor_position_to_disk_angle(dxl_id, target_position):.1f} deg).'
                    )

                    state['index'] += 1

                    if state['index'] >= len(waypoints):
                        state['done'] = True
                        self.get_logger().info(
                            f'Motor {dxl_id} sequence complete. Holding final position.'
                        )
                        continue

                    state['waypoint_start_time'] = now
                    state['last_update_time'] = now

                    next_target = waypoints[state['index']]

                    self.get_logger().info(
                        f'Motor {dxl_id} moving to next waypoint: {next_target} '
                        f'({self.motor_position_to_disk_angle(dxl_id, next_target):.1f} deg).'
                    )

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
                self.get_logger().info('Motor sequence group complete.')
                return True

            time.sleep(self.motion_loop_sleep)

        return False

    # -----------------------------
    # Joint control logic
    # -----------------------------
    def move_to_joint_targets(self, joints_deg, speed_deg_per_sec):
        motor_positions = self.joint_targets_to_motor_positions(joints_deg)

        if motor_positions is None:
            self.get_logger().error('Joint command refused.')
            return False

        sequences = {}

        for dxl_id, position in motor_positions.items():
            sequences[dxl_id] = [position]

        self.print_joint_and_disk_targets(joints_deg)

        ok = self.run_independent_motor_sequences(
            sequences,
            speed_deg_per_sec
        )

        return ok

    def update_joint_target(self, joint_name, direction):
        if joint_name not in self.joint_targets_deg:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return

        new_targets = dict(self.joint_targets_deg)
        new_angle = new_targets[joint_name] + direction * self.joint_step_deg
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
        self.get_logger().warn('Opening gripper.')
        self.set_joint_target('grip', self.grip_open_deg)

    def close_gripper(self):
        self.get_logger().warn('Closing gripper.')
        self.set_joint_target('grip', self.grip_closed_deg)

    def toggle_gripper(self):
        current_grip = self.joint_targets_deg['grip']
        halfway = 0.5 * (self.grip_open_deg + self.grip_closed_deg)

        if current_grip < halfway:
            self.open_gripper()
        else:
            self.close_gripper()

    def sweep_joint(self, joint_name):
        if joint_name not in self.joint_limits_deg:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return

        min_deg, max_deg = self.joint_limits_deg[joint_name]

        original_targets = dict(self.joint_targets_deg)

        sequence_targets = [
            dict(original_targets, **{joint_name: 0.0}),
            dict(original_targets, **{joint_name: min_deg}),
            dict(original_targets, **{joint_name: max_deg}),
            dict(original_targets, **{joint_name: 0.0}),
        ]

        for target in sequence_targets:
            ok = self.move_to_joint_targets(target, self.joint_speed_deg_per_sec)

            if not ok:
                self.get_logger().error(f'{joint_name} sweep stopped.')
                return

            self.joint_targets_deg = target

        self.get_logger().info(f'{joint_name} sweep complete.')

    # -----------------------------
    # Safety actions
    # -----------------------------
    def hold_current_positions(self):
        self.get_logger().info('Holding current positions.')

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

            self.command_position(dxl_id, present)

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

    # -----------------------------
    # UI
    # -----------------------------
    def print_instructions(self):
        print()
        print('H4HR Keyboard Control Node - JOINT CONTROL WITH dVRK COUPLING MATRIX')
        print('-------------------------------------------------------------------')
        print('Motor/Disk Mapping:')
        print('  Motor ID 1 = Disk 1')
        print('  Motor ID 2 = Disk 2')
        print('  Motor ID 3 = Disk 3')
        print('  Motor ID 4 = Disk 4')
        print()
        print('JOINT CONTROL KEYS:')
        print('  q : increase roll')
        print('  e : decrease roll')
        print('  u : increase pitch')
        print('  o : decrease pitch')
        print('  a : increase yaw')
        print('  d : decrease yaw')
        print('  j : increase grip')
        print('  l : decrease grip')
        print()
        print('GRIPPER BUTTONS:')
        print('  [ : close gripper')
        print('  ] : open gripper')
        print('  g : toggle open/close gripper')
        print()
        print('SWEEP KEYS:')
        print('  7 : sweep roll joint 0 -> min -> max -> 0')
        print('  8 : sweep pitch joint 0 -> min -> max -> 0')
        print('  9 : sweep yaw joint 0 -> min -> max -> 0')
        print()
        print('OTHER:')
        print('  +  : increase joint step by 1 deg')
        print('  -  : decrease joint step by 1 deg')
        print('  0  : return all joints to zero / gripper closed')
        print('  1  : EMERGENCY TORQUE OFF')
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
        print('HOME POSITIONS:')
        for dxl_id, home_position in zip(self.motor_ids, self.home_positions):
            print(f'  Motor {dxl_id}: {home_position}')
        print()
        print('SAFE POSITION LIMITS:')
        for dxl_id in self.motor_ids:
            min_position, max_position = self.motor_position_limits[dxl_id]
            print(f'  Motor {dxl_id}: {min_position} to {max_position}')
        print()
        print(f'Joint step: {self.joint_step_deg:.1f} deg')
        print(f'Joint speed: {self.joint_speed_deg_per_sec:.1f} deg/sec')
        print(f'Motion loop sleep: {self.motion_loop_sleep}')
        print(f'Profile velocity: {self.profile_velocity}')
        print(f'Arrival tolerance counts: {self.arrival_tolerance_counts}')
        print(f'Arrival timeout sec: {self.arrival_timeout_sec}')
        print()
        print('IMPORTANT:')
        print('  Keep one hand near motor power.')
        print('  Press 1 to disable torque immediately.')
        print('  Joint commands are converted to disk commands using the dVRK coupling matrix.')
        print('  Disk-level safety limits are still enforced.')
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
        if key == '`':
            return False

        if key == '+':
            self.increase_step()
            return True

        if key == '-':
            self.decrease_step()
            return True

        if key == '0':
            self.zero_all_joints()
            return True

        if key == '1':
            self.emergency_torque_off()
            return True

        if key == '[':
            self.close_gripper()
            return True

        if key == ']':
            self.open_gripper()
            return True

        if key == 'g':
            self.toggle_gripper()
            return True

        if key == '7':
            self.sweep_joint('roll')
            return True

        if key == '8':
            self.sweep_joint('pitch')
            return True

        if key == '9':
            self.sweep_joint('yaw')
            return True

        if key == 'q':
            self.update_joint_target('roll', +1)
            return True

        if key == 'e':
            self.update_joint_target('roll', -1)
            return True

        if key == 'u':
            self.update_joint_target('pitch', +1)
            return True

        if key == 'o':
            self.update_joint_target('pitch', -1)
            return True

        if key == 'a':
            self.update_joint_target('yaw', +1)
            return True

        if key == 'd':
            self.update_joint_target('yaw', -1)
            return True

        if key == 'j':
            self.update_joint_target('grip', +1)
            return True

        if key == 'l':
            self.update_joint_target('grip', -1)
            return True

        return True

    def run_keyboard_loop(self):
        running = True

        while rclpy.ok() and running:
            key = self.get_key()
            running = self.handle_key(key)

    def shutdown(self):
        self.emergency_torque_off()
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