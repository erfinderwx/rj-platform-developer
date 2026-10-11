# Copyright 2026 TerraAgent Robotics. SPDX-License-Identifier: Apache-2.0
"""Joystick teleoperation: joy_node + teleop_twist_joy -> cmd_vel.

    ros2 launch luchs_teleop joystick.launch.py            # gamepad on /dev/input/js0
    ros2 run teleop_twist_keyboard teleop_twist_keyboard    # keyboard alternative
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg = PathJoinSubstitution([FindPackageShare('luchs_teleop'), 'config', PythonExpression(
        ["'joystick_b.yaml' if '", LaunchConfiguration('build'), "' == 'b' else 'joystick.yaml'"])])
    return LaunchDescription([
        DeclareLaunchArgument('build', default_value='4ws',
                              description='df | fa | da | 4ws (Luchs A) or b (Luchs B: car-like scaling)'),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[cfg, {'device_id': LaunchConfiguration('joy_device_id')}]),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy',
             parameters=[cfg]),
    ])
