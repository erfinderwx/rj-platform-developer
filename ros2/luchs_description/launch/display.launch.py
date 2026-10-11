"""Display a Luchs chassis in RViz.

    ros2 launch luchs_description display.launch.py config:=4ws use_gui:=true   # Luchs A-4WS
    ros2 launch luchs_description display.launch.py config:=b                   # Luchs B
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import xacro


def _state_publisher(context):
    config = LaunchConfiguration('config').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    share = FindPackageShare('luchs_description').perform(context)
    if config == 'b':
        doc = xacro.process_file(f'{share}/urdf/luchs_b.urdf.xacro', mappings={
            'prefix': prefix, 'variant': LaunchConfiguration('variant').perform(context)})
    else:
        doc = xacro.process_file(f'{share}/urdf/luchs_a.urdf.xacro',
                                 mappings={'config': config, 'prefix': prefix})
    return [Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': ParameterValue(doc.toxml(), value_type=str)}])]


def generate_launch_description():
    pkg = FindPackageShare('luchs_description')
    use_gui = LaunchConfiguration('use_gui')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='4ws',
                              choices=['df', 'fa', 'da', '4ws', 'b'],
                              description='Luchs A build (df, fa, da, 4ws) or b for Luchs B'),
        DeclareLaunchArgument('variant', default_value='standard', choices=['standard', 'cooling'],
                              description='Luchs B finish (colour only)'),
        DeclareLaunchArgument('prefix', default_value='',
                              description='Prefix for all link and joint names'),
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Use joint_state_publisher_gui (sliders)'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz'),
        OpaqueFunction(function=_state_publisher),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(use_gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(use_gui)),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'display.rviz'])],
             output='screen', condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
