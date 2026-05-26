# UCSD MAE 156 Humanoid Robot Hand

ROS2 workspace for controlling the humanoid robot hand using Dynamixel motors and OpenRB-150.

# UCSD MAE 156 Humanoid Robot Hand

This repository contains the ROS2 software framework for controlling a humanoid robot hand tool interface using an OpenRB-150 controller and four Dynamixel motors.

The system was developed to actuate and test a robotic hand/tool mechanism for integration with a humanoid robot platform. The ROS2 package provides motor communication, motor homing, motor position feedback, manual keyboard control, and RViz-based interactive marker control.

---

## 1. System Overview

The device consists of:

- One OpenRB-150 controller
- Four Dynamixel motors
- A ROS2 computer running Ubuntu
- A custom mechanical hand/tool adapter
- Optional RViz visualization and interactive marker control

The OpenRB-150 communicates with the computer over USB. ROS2 nodes send position commands to the motors and read back motor position data.

Basic communication flow:

```text
Ubuntu Computer
    |
    | USB Serial
    v
OpenRB-150
    |
    | Dynamixel Bus
    v
Four Dynamixel Motors
    |
    v
Humanoid Robot Hand Tool Mechanism
2. Repository Structure

Expected workspace structure:

robotis_ws/
├── src/
│   └── hand_for_humanoid_robot/
│       ├── package.xml
│       ├── setup.py
│       ├── resource/
│       └── hand_for_humanoid_robot/
│           ├── keyboard_control_node.py
│           ├── auto_zero_node.py
│           ├── motor_position_publisher_node.py
│           └── rviz_control_node.py
├── README.md
└── .gitignore

The main ROS2 package is:

hand_for_humanoid_robot
3. Required Hardware

Another lab attempting to reproduce this system should prepare the following hardware:

Component	Purpose
Ubuntu computer	Runs ROS2 and control nodes
OpenRB-150 controller	Interface between computer and Dynamixel motors
4 Dynamixel motors	Actuate the tool mechanism
Dynamixel power supply	Provides motor power
USB cable	Connects OpenRB-150 to computer
Custom mechanical adapter/tool interface	Transfers motor rotation to the hand/tool mechanism
Optional RFID reader/tag	Future tool identification
Optional VR/Oculus controller	Future teleoperation input

Current motor type used in development:

Dynamixel XC330-T181-T
4. Required Software

This project was developed using:

Ubuntu 22.04
ROS2 Humble
Python 3
Dynamixel SDK
OpenRB-150 USB serial communication
RViz2

Install common ROS2 build tools:

sudo apt update
sudo apt install python3-colcon-common-extensions python3-pip git

Install the Dynamixel SDK for Python:

pip3 install dynamixel-sdk
5. Clone the Repository

Create or enter a ROS2 workspace:

mkdir -p ~/robotis_ws/src
cd ~/robotis_ws/src

Clone the repository:

git clone https://github.com/balvinbim/UCSD-MAE-156-Humanoid-Robot-Hand.git

The final structure should look like:

~/robotis_ws/src/hand_for_humanoid_robot
6. Build the ROS2 Package

From the workspace root:

cd ~/robotis_ws
colcon build --packages-select hand_for_humanoid_robot
source install/setup.bash

Optional: add the workspace source command to .bashrc:

echo "source ~/robotis_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
7. Hardware Connection Procedure

Before running any nodes:

Connect the Dynamixel motors to the OpenRB-150.
Connect the OpenRB-150 to the computer using USB.
Connect motor power.
Verify that the OpenRB-150 appears as a serial device.

Check USB devices:

ls /dev/tty*

Recommended check:

ls /dev/serial/by-id/

A stable device path may look similar to:

/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_...

If the code uses a specific serial path, update the device_name parameter in the node or launch file.

8. Motor Configuration

Each motor should have a unique Dynamixel ID.

Example expected motor IDs:

Motor 1: ID 1
Motor 2: ID 2
Motor 3: ID 3
Motor 4: ID 4

The motors should be configured using Dynamixel Wizard before running the ROS2 system.

Recommended checks:

Confirm all motors are detected.
Confirm each motor has a unique ID.
Confirm the baud rate matches the code.
Confirm the motors are in position control or extended position control mode, depending on the node.
Confirm motor directions and limits are safe for the mechanical tool.
9. Safety Warning

This system controls physical motors connected to a mechanical tool. Incorrect motor commands can damage the tool or cause injury.

Before running any control node:

Keep hands clear of the mechanism.
Start with the tool disconnected if testing software for the first time.
Verify motor IDs before sending commands.
Verify home positions before inserting the tool.
Use low speeds during initial testing.
Confirm software limits match the mechanical limits.
Be ready to disconnect motor power if the mechanism moves unexpectedly.
10. Main ROS2 Nodes

The package contains several ROS2 nodes. Each node is responsible for a different part of the control system.

To view all available nodes:

cd ~/robotis_ws
source install/setup.bash
ros2 pkg executables hand_for_humanoid_robot
10.1 Auto Zero Node
Purpose

The auto zero node moves all motors to predefined home positions. This establishes a known starting configuration before tool insertion or operation.

This node is used at the beginning of a test session to make sure the tool mechanism starts from a repeatable and safe position.

Typical Responsibilities
Connect to the OpenRB-150.
Enable motor torque.
Move all four Dynamixel motors to home positions.
Prepare the mechanism for tool insertion.
Reduce the chance of starting from an unsafe or unknown motor angle.
Example Home Positions

The home positions may be defined in the code like this:

home_positions = [2048, 0, 2048, 3072]

These values are motor encoder positions and must be adjusted to match the mechanical assembly.

Run Command
cd ~/robotis_ws
source install/setup.bash
ros2 run hand_for_humanoid_robot auto_zero_node
When to Use

Use this node:

Before inserting the tool.
Before starting keyboard control.
Before starting RViz control.
After powering on the motors.
After any unexpected movement or reset.
Expected Result

All motors should move to their defined home positions and stop.

10.2 Keyboard Control Node
Purpose

The keyboard control node allows manual control of the motors using keyboard inputs. This is useful for early testing, debugging, and checking individual motor movement.

Typical Responsibilities
Connect to the OpenRB-150.
Enable torque on the Dynamixel motors.
Receive keyboard commands from the user.
Convert key presses into motor position changes.
Send position commands to the motors.
Keep motion within defined software limits.
Run Command
cd ~/robotis_ws
source install/setup.bash
ros2 run hand_for_humanoid_robot keyboard_control_node
When to Use

Use this node:

To test one motor at a time.
To verify that each motor responds correctly.
To check the relationship between motor movement and tool movement.
To manually move the tool during bench testing.
Expected Result

Pressing assigned keys should move the selected motors in small controlled increments.

Notes for New Labs

Before using this node with the tool attached, test with the motors unloaded or with the mechanism disconnected. Confirm that positive and negative motion directions match the mechanical design.

10.3 Motor Position Publisher Node
Purpose

The motor position publisher node reads the current encoder position of each Dynamixel motor and publishes the positions to a ROS2 topic.

This is useful for debugging, calibration, and checking whether a motor is offset from its expected home position.

Typical Responsibilities
Connect to the OpenRB-150.
Read present position values from all motors.
Publish motor positions to a ROS2 topic.
Allow other ROS2 nodes or users to monitor motor feedback.
Run Command
cd ~/robotis_ws
source install/setup.bash
ros2 run hand_for_humanoid_robot motor_position_publisher_node
View Published Data

In another terminal:

cd ~/robotis_ws
source install/setup.bash
ros2 topic list

Then echo the motor position topic. The topic name depends on the code, but it may be:

ros2 topic echo /motor_positions
When to Use

Use this node:

To check current motor positions.
To compare actual positions with home positions.
To debug a motor that appears offset.
To verify that encoder feedback is working.
To record motor positions during testing.
Expected Result

The terminal should display motor position values for each motor.

10.4 RViz Interactive Marker Control Node
Purpose

The RViz interactive marker control node allows a user to control the motors visually through RViz. Interactive marker rings are rotated in RViz, and the node converts marker rotation into motor position commands.

This is intended to provide a more intuitive control interface than keyboard control.

Typical Responsibilities
Create interactive markers in RViz.
Detect marker rotation.
Convert marker rotation to motor target positions.
Send updated position commands to Dynamixel motors.
Provide a visual interface for testing tool movement.
Run Command

Terminal 1:

cd ~/robotis_ws
source install/setup.bash
ros2 run hand_for_humanoid_robot rviz_control_node

Terminal 2:

rviz2
When to Use

Use this node:

To test visual control of the hand/tool.
To debug the relationship between marker rotation and motor angle.
To prepare for future teleoperation mapping.
To demonstrate the control interface.
Expected Result

RViz should show interactive markers. Rotating the marker rings should command motor movement.

Notes for New Labs

The current marker-to-motor relationship may require calibration. A future improvement is to map one full marker rotation to the full safe joint limit of the corresponding motor/tool axis.

11. Recommended Startup Procedure

A new lab should use the following startup order.

Step 1: Connect Hardware

Connect:

Computer → USB → OpenRB-150 → Dynamixel Motors → Tool Mechanism

Make sure motor power is connected.

Step 2: Build and Source
cd ~/robotis_ws
colcon build --packages-select hand_for_humanoid_robot
source install/setup.bash
Step 3: Confirm ROS2 Can See the Package
ros2 pkg list | grep hand_for_humanoid_robot
Step 4: Check Available Nodes
ros2 pkg executables hand_for_humanoid_robot
Step 5: Run Auto Zero
ros2 run hand_for_humanoid_robot auto_zero_node
Step 6: Check Motor Positions

Terminal 1:

ros2 run hand_for_humanoid_robot motor_position_publisher_node

Terminal 2:

source ~/robotis_ws/install/setup.bash
ros2 topic echo /motor_positions
Step 7: Run a Control Node

For keyboard control:

ros2 run hand_for_humanoid_robot keyboard_control_node

For RViz control:

ros2 run hand_for_humanoid_robot rviz_control_node
12. Common ROS2 Commands
Build Package
cd ~/robotis_ws
colcon build --packages-select hand_for_humanoid_robot
source install/setup.bash
List Nodes
ros2 node list
List Topics
ros2 topic list
Echo a Topic
ros2 topic echo /topic_name
Check Package Executables
ros2 pkg executables hand_for_humanoid_robot
Stop a Running Node

Press:

Ctrl + C

in the terminal running the node.

13. Troubleshooting
Problem: ROS2 Cannot Find the Package

Run:

cd ~/robotis_ws
colcon build --packages-select hand_for_humanoid_robot
source install/setup.bash

Then check:

ros2 pkg list | grep hand_for_humanoid_robot
Problem: Node Does Not Run

Check the node names:

ros2 pkg executables hand_for_humanoid_robot

Use the exact executable name shown by the command.

Problem: Motors Do Not Move

Check:

OpenRB-150 is connected over USB.
Motor power is connected.
Dynamixel IDs are correct.
Baud rate matches the code.
Device path is correct.
Torque is enabled.
Motor limits are not preventing motion.
Problem: Permission Denied on USB Port

Try:

sudo usermod -a -G dialout $USER

Then log out and log back in.

Problem: Wrong Motor Moves

Check motor IDs using Dynamixel Wizard. The ID in the code must match the physical motor.

Problem: Motor Moves in Wrong Direction

This may be caused by motor orientation or software sign convention. Update the direction mapping in the node code after confirming the mechanical direction is safe.

Problem: RViz Marker Moves but Motor Does Not Respond

Check:

The RViz node is running.
The OpenRB-150 is connected.
The marker topic is active.
The motor command function is being called.
The device path and motor IDs are correct.
14. Development Notes

Known future improvements:

Improve the relationship between RViz marker rotation and motor position.
Map one full marker rotation to each motor joint limit.
Increase motor speed carefully after safety testing.
Improve real-time response so new commands interrupt old target motions.
Integrate Oculus or VR controller input.
Add RFID-based tool identification.
Add safer launch files and parameter files.
Add calibration documentation for each motor/tool axis.
15. Updating GitHub

After modifying code or documentation:

cd ~/robotis_ws
git status
git add src/hand_for_humanoid_robot README.md .gitignore
git commit -m "Update ROS2 hand documentation"
git push

If there are no changes, Git will say:

nothing to commit, working tree clean
16. Important Files Not Included in GitHub

The following folders should not be uploaded because they are generated by ROS2:

build/
install/
log/

These can be recreated with:

colcon build

The .gitignore file should include:

build/
install/
log/
__pycache__/
*.pyc
.DS_Store
.vscode/

After pasting, save in nano:

```text
Ctrl + O
Enter
Ctrl + X

Then push the updated README:

cd ~/robotis_ws
git add README.md
git commit -m "Add from-scratch ROS2 hand setup manual"
git push
