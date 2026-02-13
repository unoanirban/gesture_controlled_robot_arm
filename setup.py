from setuptools import setup
import os
from glob import glob

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.1.0',
    description='Hand-controlled 4-DOF arm controller',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # URDF and launch files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    keywords=['ROS2'],
    classifiers=[
        'Programming Language :: Python',
        'License :: OSI Approved :: Apache Software License',
    ],
    entry_points={
        'console_scripts': [
            'hand_detector = robot_controller.hand_detector:main',
            'arm_commander = robot_controller.arm_commander:main',
        ],
    },
)

