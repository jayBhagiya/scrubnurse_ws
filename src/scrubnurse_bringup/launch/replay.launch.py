"""
Play a recorded run (launched with record:=true) into the dashboard.

    ros2 launch scrubnurse_bringup replay.launch.py run:=$HOME/scrubnurse_runs/<date_time>

The dashboard on http://localhost:8080 shows a REPLAY badge, its controls are off and a
timeline pauses, jumps and changes the speed; RViz shows the arm (rviz:=false to skip it). At
the end the run starts over, paused. The replay runs on its own ROS domain, on this PC only,
and plays the instrument requests under another name, so a running cell never receives them.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
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
        DeclareLaunchArgument('rviz', default_value='true', description='Show the arm in RViz'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', arg('domain')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        # --clock: the dashboard's timeline and RViz follow the recording's time.
        ExecuteProcess(cmd=['ros2', 'bag', 'play', arg('run'), '--clock', '20', '--loop',
                            '--remap', '/tool_request:=/replayed/tool_request'],
                       output='screen'),
        Node(package='image_transport', executable='republish', arguments=['compressed', 'raw'],
             remappings=[('in/compressed', RECORDED_CAMERA + '/compressed'),
                         ('out', '/replay/image')]),
        Node(package='scrubnurse_dashboard', executable='dashboard', output='screen',
             parameters=[PathJoinSubstitution(
                 [FindPackageShare('scrubnurse_bringup'), 'config', 'scrubnurse.yaml']), {
                     'mode': 'replay',
                     'run': arg('run'),
                     'port': ParameterValue(arg('dashboard_port'), value_type=int),
                 }],
             remappings=[('image', '/replay/image')]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', PathJoinSubstitution(
                 [FindPackageShare('scrubnurse_bringup'), 'config', 'replay.rviz'])],
             parameters=[{'use_sim_time': True}], condition=IfCondition(arg('rviz'))),
    ])
