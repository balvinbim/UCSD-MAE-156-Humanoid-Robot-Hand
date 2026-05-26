import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hand_for_humanoid_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
   	(os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
	(os.path.join('share', package_name,'rviz'), glob('rviz/*rviz')), 
       # This line installs all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],

    install_requires=['setuptools', 'dynamixel-sdk'],
    zip_safe=True,
    maintainer='arclab',
    maintainer_email='ssylvester@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'auto_zero_node = hand_for_humanoid_robot.auto_zero_node:main',
		'keyboard_control_node = hand_for_humanoid_robot.keyboard_control_node:main',
		'interactive_marker_control_node = hand_for_humanoid_robot.interactive_marker_control_node:main',
		'dynamixel_controller_node = hand_for_humanoid_robot.dynamixel_controller_node:main',
		'raw_joint_keyboard_node = hand_for_humanoid_robot.raw_joint_keyboard_node:main',
		'motor_current_monitor_node = hand_for_humanoid_robot.motor_current_monitor_node:main',
        ],
    },
)
