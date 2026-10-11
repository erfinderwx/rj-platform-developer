# Copyright 2026 TerraAgent Robotics. SPDX-License-Identifier: Apache-2.0
"""Bring up a Luchs A chassis: robot model, CAN driver and (optionally) joystick teleoperation.

    ros2 launch luchs_bringup luchs_a.launch.py build:=4ws can_interface:=can0
    ros2 launch luchs_bringup luchs_a.launch.py build:=4ws dry_run:=true      # no hardware
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

BUILDS = ('df', 'fa', 'da', '4ws')


def _nodes(context):
    build = LaunchConfiguration('build').perform(context)
    if build not in BUILDS:
        raise RuntimeError(f"build must be one of {', '.join(BUILDS)} (got '{build}')")
    params_file = LaunchConfiguration('params_file').perform(context) or PathJoinSubstitution(
        [FindPackageShare('luchs_driver'), 'config', f'luchs_a_{build}.yaml']).perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)

    description = Command([
        'xacro ', PathJoinSubstitution(
            [FindPackageShare('luchs_description'), 'urdf', 'luchs_a.urdf.xacro']),
        f' config:={build}', f' prefix:={prefix}'])

    return [
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': ParameterValue(description, value_type=str)}]),
        Node(
            package='luchs_driver', executable='luchs_driver_node', name='luchs_driver',
            output='screen', emulate_tty=True,
            parameters=[params_file, {
                'build': build,
                'can_interface': LaunchConfiguration('can_interface'),
                'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
                'odom.publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                'odom.frame_id': f'{prefix}odom',
                'odom.child_frame_id': f'{prefix}base_footprint',
                'joint_prefix': prefix,
            }]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('build', default_value='4ws', description='df | fa | da | 4ws'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('prefix', default_value='', description='TF / joint name prefix'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Driver parameters; default config/luchs_a_<build>.yaml'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='false when robot_localization publishes odom -> base'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true: no CAN socket, for testing without hardware'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='start joystick teleoperation'),
        OpaqueFunction(function=_nodes),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('luchs_teleop'), 'launch', 'joystick.launch.py'])),
            launch_arguments={'build': LaunchConfiguration('build')}.items(),
            condition=IfCondition(LaunchConfiguration('teleop'))),
    ])
