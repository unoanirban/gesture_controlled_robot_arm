# robot_controller/launch/bringup.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_controller')
    urdf_path = os.path.join(pkg_share, 'urdf', 'four_dof_arm_side_joints.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # robot_state_publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # joint_state_publisher (provides joint state topic if needed)
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    hand_node = Node(
        package='robot_controller',
        executable='hand_detector',
        name='hand_detector',
        output='screen'
    )

    arm_node = Node(
        package='robot_controller',
        executable='arm_commander',
        name='arm_commander',
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        jsp_node,
        hand_node,
        arm_node
    ])
