# UCSD MAE 156 Humanoid Robot Hand

This repository contains the ROS2 Humble software framework for controlling a humanoid robot hand tool interface using an OpenRB-150 controller, four Dynamixel XC330-T181-T motors, RViz control, keyboard control, auto-zeroing, and future RFID-based tool detection.

The system was developed for a custom humanoid robot hand adapter designed to actuate da Vinci Research Kit / dVRK-style tool mechanisms. The software converts desired tool commands such as roll, pitch, yaw, and grip into Dynamixel motor positions using a coupling matrix.

---

## 1. System Overview

The system consists of:

* Ubuntu computer running ROS2 Humble
* OpenRB-150 controller
* Four Dynamixel XC330-T181-T motors
* Custom mechanical hand/tool adapter
* RViz2 interactive control interface
* Standalone numpad keyboard control
* Auto-zero startup sequence
* Optional PN532 RFID reader for future tool identification

Basic communication flow:

```text
Ubuntu ROS2 Computer
        |
        | USB Serial
        v
OpenRB-150 Controller
        |
        | Dynamixel Bus
        v
Four Dynamixel Motors
        |
        v
Humanoid Robot Hand / dVRK Tool Adapter
```

The ROS2 system sends tool-level commands such as roll, pitch, yaw, and grip. These commands are converted into individual motor/disk commands using the dVRK coupling matrix.

---

## 2. Main Repository Structure

Expected ROS2 workspace structure:

```text
robotis_ws/
├── src/
│   └── hand_for_humanoid_robot/
│       ├── package.xml
│       ├── setup.py
│       ├── launch/
│       │   └── rviz_orientation_control.launch.py
│       ├── rviz/
│       │   └── rviz_orientation_control.rviz
│       └── hand_for_humanoid_robot/
│           ├── __init__.py
│           ├── auto_zero_node.py
│           ├── dynamixel_controller_node.py
│           ├── hand_for_humanoid_robot_ui_node.py
│           ├── joint_command_bridge_node.py
│           ├── keyboard_control_node.py
│           ├── rfid_detection_node.py
│           └── rviz_orientation_control_node.py
├── build/
├── install/
└── log/
```

The main ROS2 package is:

```text
hand_for_humanoid_robot
```

---

## 3. Required Hardware

| Component                       | Purpose                                            |
| ------------------------------- | -------------------------------------------------- |
| Ubuntu computer                 | Runs ROS2 Humble and control nodes                 |
| OpenRB-150                      | USB-to-Dynamixel controller                        |
| 4 Dynamixel XC330-T181-T motors | Actuate roll, pitch, yaw, and grip disks           |
| Dynamixel power supply          | Provides motor power                               |
| USB cable                       | Connects OpenRB-150 to the computer                |
| Custom tool adapter             | Couples motors to the da Vinci/dVRK tool interface |
| PN532 RFID reader               | Future tool identification                         |
| RFID/NFC stickers               | Future tool ID tags                                |

---

## 4. Required Software

This project was developed using:

```text
Ubuntu 22.04
ROS2 Humble
Python 3
Dynamixel SDK
RViz2
OpenRB-150 USB serial communication
```

Install ROS2 build tools:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-pip git
```

Install the Dynamixel SDK for Python:

```bash
pip3 install dynamixel-sdk
```

---

## 5. Clone the Repository

Create or enter the ROS2 workspace:

```bash
mkdir -p ~/robotis_ws/src
cd ~/robotis_ws/src
```

Clone the repository:

```bash
git clone https://github.com/balvinbim/UCSD-MAE-156-Humanoid-Robot-Hand.git hand_for_humanoid_robot
```

Expected final path:

```text
~/robotis_ws/src/hand_for_humanoid_robot
```

---

## 6. Build the ROS2 Package

From the workspace root:

```bash
cd ~/robotis_ws
colcon build --packages-select hand_for_humanoid_robot
source install/setup.bash
```

Optional: source the workspace automatically when opening a terminal:

```bash
echo "source ~/robotis_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Check that ROS2 sees the package:

```bash
ros2 pkg list | grep hand_for_humanoid_robot
```

Check available executables:

```bash
ros2 pkg executables hand_for_humanoid_robot
```

---

## 7. OpenRB-150 and Motor Setup

Connect the hardware in this order:

```text
Computer USB -> OpenRB-150 -> Dynamixel bus -> 4 Dynamixel motors
```

Then connect motor power.

Check that the OpenRB appears on Ubuntu:

```bash
ls /dev/ttyACM*
ls /dev/serial/by-id/
```

A stable device path may look similar to:

```text
/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F42208A55157375037202020FF122F34-if00
```

The current launch file uses this stable `/dev/serial/by-id/` path by default. If another OpenRB is used, update the `device_name` launch argument or node parameter.

---

## 8. Motor Configuration

Expected motor IDs:

| Motor   | Dynamixel ID | Function                    |
| ------- | -----------: | --------------------------- |
| Motor 1 |            1 | Disk 1 / Roll-related disk  |
| Motor 2 |            2 | Disk 2 / Pitch-related disk |
| Motor 3 |            3 | Disk 3 / Yaw + Grip disk    |
| Motor 4 |            4 | Disk 4 / Yaw + Grip disk    |

Before running ROS2, use Dynamixel Wizard to verify:

* Each motor is detected
* Each motor has a unique ID
* The baud rate is 57600
* The motors respond to position commands
* The OpenRB-150 is flashed with the USB-to-Dynamixel sketch
* The mechanical limits are safe

---

## 9. Safety Warning

This project controls physical motors connected to a mechanical tool interface. Incorrect commands can damage the mechanism or cause injury.

Before running any control node:

* Keep hands clear of the mechanism
* Keep one hand near the motor power switch
* Verify motor IDs before commanding motion
* Verify home positions before inserting the tool
* Use low speed during initial testing
* Confirm software limits match the physical mechanism
* Do not run two nodes that use the OpenRB serial port at the same time
* Press emergency stop or disconnect power if motion is unexpected

The OpenRB serial port can only be owned by one motor-control node at a time.

Do not run these together:

```text
auto_zero_node
dynamixel_controller_node
keyboard_control_node
rfid_detection_node
Dynamixel Wizard
Arduino Serial Monitor
```

---

## 10. Current Main Nodes

### 10.1 `auto_zero_node.py`

Purpose:

```text
Runs the startup homing and tool preparation sequence.
```

Responsibilities:

* Opens the OpenRB/Dynamixel serial port
* Waits for no-tool confirmation
* Homes the motors
* Waits for tool insertion confirmation
* Runs the auto-zero sweep
* Publishes auto-zero status
* Exits after successful auto-zero so the main controller can start

Main topics:

```text
Subscribes:
  /h4hr/confirm_no_tool
  /h4hr/confirm_tool_insertion
  /h4hr/emergency_stop

Publishes:
  /h4hr/auto_zero_status
  /h4hr/auto_zero_ready
  /joint_states
```

Run alone:

```bash
cd ~/robotis_ws
source install/setup.bash
ros2 run hand_for_humanoid_robot auto_zero_node
```

---

### 10.2 `dynamixel_controller_node.py`

Purpose:

```text
Main physical motor controller for normal RViz operation.
```

Responsibilities:

* Opens the OpenRB/Dynamixel serial port
* Subscribes to cleaned joint commands
* Converts roll, pitch, yaw, and grip into disk angles
* Converts disk angles into Dynamixel goal positions
* Commands motors 1 through 4
* Publishes joint states
* Handles enable/disable and emergency stop behavior

Main topics:

```text
Subscribes:
  /h4hr/joint_command
  /h4hr/enable_control
  /h4hr/estop

Publishes:
  /joint_states
```

Run alone:

```bash
cd ~/robotis_ws
source install/setup.bash
ros2 run hand_for_humanoid_robot dynamixel_controller_node
```

---

### 10.3 `rviz_orientation_control_node.py`

Purpose:

```text
Creates the RViz interactive control marker for tool roll, pitch, yaw, and grip.
```

Responsibilities:

* Creates the RViz orientation marker
* Creates the gripper control marker
* Stores the current desired roll/pitch/yaw/grip command
* Publishes target tool joint commands

Main topic:

```text
Publishes:
  /target_tool_joints
```

This node does not directly command the motors. It sends target joint commands to the bridge node.

---

### 10.4 `joint_command_bridge_node.py`

Purpose:

```text
Converts RViz target tool joints into the command format used by the motor controller.
```

Responsibilities:

* Subscribes to `/target_tool_joints`
* Converts radians to degrees internally
* Clamps roll, pitch, yaw, and grip to safe limits
* Publishes cleaned commands to `/h4hr/joint_command`

Main topics:

```text
Subscribes:
  /target_tool_joints

Publishes:
  /h4hr/joint_command
```

---

### 10.5 `hand_for_humanoid_robot_ui_node.py`

Purpose:

```text
Simple terminal UI for startup confirmations and reset commands.
```

Keyboard options:

```text
n = confirm no tool inserted
y = confirm tool inserted
r = reset RViz command and motor command to home
? = show menu
q = quit UI node
```

Main topics:

```text
Publishes:
  /h4hr/confirm_no_tool
  /h4hr/confirm_tool_insertion
  /target_tool_joints
  /h4hr/joint_command
```

Run separately:

```bash
cd ~/robotis_ws
source install/setup.bash
ros2 run hand_for_humanoid_robot hand_for_humanoid_robot_ui_node
```

---

### 10.6 `keyboard_control_node.py`

Purpose:

```text
Standalone direct keyboard/numpad control for bench testing.
```

This node directly opens the OpenRB/Dynamixel serial port and commands the motors. It should not run at the same time as the main RViz launch or the Dynamixel controller node.

Current numpad mapping:

```text
7 = forward + left + roll left
8 = forward
9 = forward + right + roll right

4 = left / yaw left
5 = hold current position
6 = right / yaw right

1 = backward + left + roll left
2 = backward
3 = backward + right + roll right

0 = return all joints to zero / home command
. = toggle gripper open/close

+ = increase joint step
- = decrease joint step
e = emergency torque off
` = quit
```

Important current behavior:

* Motors 3 and 4 are direction-corrected in software
* Grip direction is inverted for the current mechanical setup
* Close command goes farther than open to improve tool closing

Run separately:

```bash
cd ~/robotis_ws
source install/setup.bash
ros2 run hand_for_humanoid_robot keyboard_control_node
```

Make sure Num Lock is ON.

---

### 10.7 `rfid_detection_node.py`

Purpose:

```text
Future tool identification using a PN532 RFID/NFC reader connected through the OpenRB.
```

Current intended flow:

```text
RFID sticker -> PN532 -> OpenRB-150 -> USB serial -> ROS2 RFID node
```

This node is not currently part of the main launch sequence. It should not run at the same time as motor-control nodes unless the OpenRB sketch and serial communication strategy are designed to support both.

---

## 11. Main Launch File

The current main launch file is:

```text
rviz_orientation_control.launch.py
```

Location:

```text
~/robotis_ws/src/hand_for_humanoid_robot/launch/rviz_orientation_control.launch.py
```

It launches:

```text
auto_zero_node
static_transform_publisher
rviz_orientation_control_node
joint_command_bridge_node
dynamixel_controller_node
rviz2
```

When `run_auto_zero:=true`, the launch file starts `auto_zero_node` first. After auto-zero exits successfully, the main control nodes start. This prevents `auto_zero_node` and `dynamixel_controller_node` from fighting over the same OpenRB serial port.

Run with auto-zero:

```bash
cd ~/robotis_ws
source install/setup.bash
ros2 launch hand_for_humanoid_robot rviz_orientation_control.launch.py
```

Run without auto-zero:

```bash
cd ~/robotis_ws
source install/setup.bash
ros2 launch hand_for_humanoid_robot rviz_orientation_control.launch.py run_auto_zero:=false
```

Run without RViz:

```bash
cd ~/robotis_ws
source install/setup.bash
ros2 launch hand_for_humanoid_robot rviz_orientation_control.launch.py use_rviz:=false
```

---

## 12. Recommended Startup Procedure

### Step 1: Connect hardware

```text
Computer -> OpenRB-150 -> Dynamixel motors -> Tool adapter
```

Then connect motor power.

### Step 2: Check the serial port

```bash
ls /dev/ttyACM*
ls /dev/serial/by-id/
```

### Step 3: Build and source

```bash
cd ~/robotis_ws
colcon build --packages-select hand_for_humanoid_robot
source install/setup.bash
```

### Step 4: Start the UI node in a separate terminal

```bash
cd ~/robotis_ws
source install/setup.bash
ros2 run hand_for_humanoid_robot hand_for_humanoid_robot_ui_node
```

### Step 5: Start the main launch

In another terminal:

```bash
cd ~/robotis_ws
source install/setup.bash
ros2 launch hand_for_humanoid_robot rviz_orientation_control.launch.py
```

### Step 6: Use UI confirmations

In the UI terminal:

```text
n = confirm no tool inserted
y = confirm tool inserted
r = reset home
```

### Step 7: Control with RViz

RViz should open with the fixed frame set to `world` and the interactive marker display loaded.

Use the RViz marker to command tool orientation and grip.

---

## 13. Standalone Keyboard Testing Procedure

Use this only when the main launch is not running.

Stop other ROS2 nodes first:

```bash
pkill -f ros2
pkill -f rviz2
```

Then run:

```bash
cd ~/robotis_ws
source install/setup.bash
ros2 run hand_for_humanoid_robot keyboard_control_node
```

Make sure Num Lock is ON.

Emergency stop:

```text
e = torque off
```

Quit:

```text
` = quit
```

---

## 14. Common ROS2 Commands

Build package:

```bash
cd ~/robotis_ws
colcon build --packages-select hand_for_humanoid_robot
source install/setup.bash
```

List nodes:

```bash
ros2 node list
```

List topics:

```bash
ros2 topic list
```

Echo a topic:

```bash
ros2 topic echo /topic_name
```

Check package executables:

```bash
ros2 pkg executables hand_for_humanoid_robot
```

Check who is publishing/subscribing to a topic:

```bash
ros2 topic info /topic_name -v
```

Stop a running node:

```text
Ctrl + C
```

---

## 15. Troubleshooting

### Problem: ROS2 cannot find the package

Run:

```bash
cd ~/robotis_ws
colcon build --packages-select hand_for_humanoid_robot
source install/setup.bash
ros2 pkg list | grep hand_for_humanoid_robot
```

### Problem: Node does not run

Check the exact executable name:

```bash
ros2 pkg executables hand_for_humanoid_robot
```

### Problem: Motors do not move

Check:

* OpenRB-150 is connected over USB
* Motor power is connected
* OpenRB has the correct USB-to-Dynamixel sketch
* No other process is using the serial port
* Dynamixel IDs are correct
* Baud rate is 57600
* Device path is correct
* Torque is enabled
* Software limits are not blocking motion

Check if another program owns the port:

```bash
lsof /dev/ttyACM0
```

or:

```bash
lsof /dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F42208A55157375037202020FF122F34-if00
```

### Problem: Dynamixel Wizard cannot find motors

Make sure no ROS2 node is using the OpenRB serial port.

Stop ROS2 nodes:

```bash
pkill -f ros2
pkill -f rviz2
```

Then reopen Dynamixel Wizard and scan again.

### Problem: Permission denied on USB port

Run:

```bash
sudo usermod -a -G dialout $USER
```

Then log out and log back in.

### Problem: Wrong motor moves

Check motor IDs in Dynamixel Wizard. The code assumes:

```text
Motor 1 = ID 1
Motor 2 = ID 2
Motor 3 = ID 3
Motor 4 = ID 4
```

### Problem: Motor moves in the wrong direction

Motor direction may need to be corrected in software. The current keyboard control node includes direction correction for motors 3 and 4.

### Problem: RViz marker moves but motors do not respond

Check:

* `rviz_orientation_control_node` is running
* `joint_command_bridge_node` is running
* `dynamixel_controller_node` is running
* OpenRB serial port is available
* `/target_tool_joints` is changing
* `/h4hr/joint_command` is changing

Useful commands:

```bash
ros2 topic echo /target_tool_joints
ros2 topic echo /h4hr/joint_command
```

### Problem: UI reset works briefly, then motors return to old position

The RViz orientation control node may still be storing and republishing the old command. A future improvement is to add a dedicated reset subscriber to `rviz_orientation_control_node.py` so its internal roll/pitch/yaw/grip values reset too.

---

## 16. GitHub Update Commands

After changing code or documentation:

```bash
cd ~/robotis_ws
git status
git add src/hand_for_humanoid_robot README.md .gitignore
git commit -m "Update ROS2 humanoid hand README"
git push
```

If Git says there are no changes:

```text
nothing to commit, working tree clean
```

then the repository is already up to date.

---

## 17. Files and Folders Not to Commit

Do not upload generated ROS2 build folders:

```text
build/
install/
log/
```

Recommended `.gitignore`:

```text
build/
install/
log/
__pycache__/
*.pyc
.DS_Store
.vscode/
```

---

## 18. Current Development Notes

Current working features:

* Auto-zero startup sequence
* RViz orientation control
* Joint command bridge
* Dynamixel motor controller
* Standalone numpad keyboard control
* Terminal UI confirmations
* Software joint bounds
* Disk-level safety bounds
* Motor 3 and 4 direction correction
* Gripper open/close testing

Future improvements:

* Fully integrate RFID detection into launch sequence
* Add reset subscriber inside RViz orientation control node
* Improve gripper calibration
* Document final motor home positions
* Add parameter YAML files
* Add launch option for UI node
* Add current sensing / motor load monitoring
* Add VR/Oculus teleoperation input
* Improve tool-specific calibration by RFID tag
