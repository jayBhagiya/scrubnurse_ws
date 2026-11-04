"""
Play a recorded run (launched with record:=true) into the dashboard.

    ros2 launch scrubnurse_bringup replay.launch.py run:=$HOME/scrubnurse_runs/<date_time>

The dashboard on http://localhost:8080 shows a REPLAY badge and its controls are off. The
replay runs on its own ROS domain, on this PC only, and plays the instrument requests under
another name, so a running cell never receives them.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

RECORDED_CAMERA = '/recording/camera'  # as in scrubnurse.launch.py


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('run', description='Folder of the recorded run'),
        DeclareLaunchArgument('domain', default_value='99', description='ROS domain to replay on'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', arg('domain')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        ExecuteProcess(cmd=['ros2', 'bag', 'play', arg('run'),
                            '--remap', '/tool_request:=/replayed/tool_request'],
                       output='screen'),
        Node(package='image_transport', executable='republish', arguments=['compressed', 'raw'],
             remappings=[('in/compressed', RECORDED_CAMERA + '/compressed'),
                         ('out', '/replay/image')]),
        Node(package='scrubnurse_dashboard', executable='dashboard', output='screen',
             parameters=[PathJoinSubstitution(
                 [FindPackageShare('scrubnurse_bringup'), 'config', 'scrubnurse.yaml']), {
                     'mode': 'replay',
                     'port': ParameterValue(arg('dashboard_port'), value_type=int),
                 }],
             remappings=[('image', '/replay/image')]),
    ])
