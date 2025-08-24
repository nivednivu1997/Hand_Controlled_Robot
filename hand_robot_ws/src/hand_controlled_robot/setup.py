import os
from glob import glob
from setuptools import setup

package_name = 'hand_controlled_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('hand_controlled_robot', 'launch', '*launch.[pxy][yma]*'))),
    ],
    zip_safe=True,
    maintainer='thedush',
    maintainer_email='nived@thedush.com',
    description='Hand gesture controlled robot using MediaPipe and ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_controlled_robot_node = hand_controlled_robot.hand_controlled_robot_node:main',
        ],
    },
    install_requires=[
        'setuptools',
        'opencv-python',
        'mediapipe',
    ],
)
