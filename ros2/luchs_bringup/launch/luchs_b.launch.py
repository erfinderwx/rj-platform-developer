# Modified 2026-10-11 for 1.1.0-preview.1: B defaults to simulation.
# Copyright 2026 TerraAgent Robotics. SPDX-License-Identifier: Apache-2.0
"""Bring up a Luchs B chassis: robot model, CAN driver and (optionally) joystick teleoperation.

    ros2 launch luchs_bringup luchs_b.launch.py can_interface:=can0          # simulator by default; LCI firmware unconfirmed
    ros2 launch luchs_bringup luchs_b.launch.py protocol:=legacy_b           # VCU reference simulator; hardware interface requires confirmation
    ros2 launch luchs_bringup luchs_b.launch.py dry_run:=true                # no hardware
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PROTOCOLS = ('lci', 'legacy_b')


def _nodes(context):
    protocol = LaunchConfiguration('protocol').perform(context)
    if protocol not in PROTOCOLS:
        raise RuntimeError(f"protocol must be one of {', '.join(PROTOCOLS)} (got '{protocol}')")
    params_file = LaunchConfiguration('params_file').perform(context) or PathJoinSubstitution(
        [FindPackageShare('luchs_driver'), 'config', 'luchs_b.yaml']).perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)

    description = Command([
        'xacro ', PathJoinSubstitution(
            [FindPackageShare('luchs_description'), 'urdf', 'luchs_b.urdf.xacro']),
        f" variant:={LaunchConfiguration('variant').perform(context)}", f' prefix:={prefix}'])

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
                'build': 'b',
                'protocol.type': protocol,
                'control.rate': 100.0 if protocol == 'legacy_b' else 50.0,
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
        DeclareLaunchArgument('protocol', default_value='lci',
                              description='lci (draft codec/simulator) | legacy_b (VCU V2.6 reference codec/simulator)'),
        DeclareLaunchArgument('variant', default_value='standard',
                              description='standard | cooling (model colour only)'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('prefix', default_value='', description='TF / joint name prefix'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Driver parameters; default config/luchs_b.yaml'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='false when robot_localization publishes odom -> base'),
        DeclareLaunchArgument('dry_run', default_value='true',
                              description='preview default true: simulator; hardware requires explicit false after confirmation'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='start joystick teleoperation'),
        OpaqueFunction(function=_nodes),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('luchs_teleop'), 'launch', 'joystick.launch.py'])),
            launch_arguments={'build': 'b'}.items(),
            condition=IfCondition(LaunchConfiguration('teleop'))),
    ])
