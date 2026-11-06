"""
Scrubnurse application nodes: tool perception, voice commands, the task controller and the dashboard.

Expects the robot side (UR driver or simulation, plus MoveIt 2) and a camera
image topic to be up; real.launch.py, mock.launch.py and sim.launch.py include this file.

record:=true saves the run to <record_dir>/<date_time>/ (MCAP, camera as JPEG) for
`ros2 run scrubnurse_dashboard metrics` and replay.launch.py.
"""

import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def config(name):
    return PathJoinSubstitution([FindPackageShare('scrubnurse_bringup'), 'config', name])


ARGUMENTS = [
    ('params_file', config('scrubnurse.yaml'), 'Parameters of all Scrubnurse nodes'),
    ('override_params_file', LaunchConfiguration('params_file'),
     'Parameters loaded after params_file (e.g. sim.yaml)'),
    ('poses_file', config('poses.yaml'), 'Named robot poses'),
    ('image_topic', '/od_camera/stream/rgb', 'Camera image topic for the tool detector'),
    ('gripper_backend', 'robotiq_socket', 'Gripper backend: robotiq_socket, action or mock'),
    ('robot_ip', '', 'UR controller address, where the Robotiq URCap socket listens'),
    ('home_pose', 'HD', 'Rest pose: joint move here before the first task (empty: off)'),
    ('use_sim_time', 'false', 'Use the /clock topic (simulation)'),
    ('detector', 'true', 'Start the YOLO tool detector'),
    ('localizer', 'true', 'Start the tool localizer (pixels to robot coordinates)'),
    ('voice', 'true', 'Start the voice command node'),
    ('dashboard', 'true', 'Start the web dashboard'),
    ('dashboard_mode', 'real', 'Shown on the dashboard: real, sim or mock'),
    ('dashboard_host', '127.0.0.1',
     'Dashboard address; 0.0.0.0 makes it reachable from the network (no login)'),
    ('dashboard_port', '8080', 'Dashboard port'),
    ('record', 'false', 'Record the run (see record_dir)'),
    ('record_dir', os.path.expanduser('~/scrubnurse_runs'), 'Folder for the recorded runs'),
]

RECORDED_CAMERA = '/recording/camera'  # replay.launch.py plays it back from here
RECORDED_TOPICS = [
    '/scrubnurse_controller/cycle', '/scrubnurse_controller/status', '/tool_request',
    '/tool_detections_2d', '/tool_detections', '/voice_command/events', '/voice_command/status',
    '/voice_command/transcript', '/kinect_camera/status', '/joint_states', '/tf', '/tf_static',
    '/robot_description', '/rosout', RECORDED_CAMERA + '/compressed',
]


def generate_launch_description():
    arg = LaunchConfiguration
    params = [arg('params_file'), arg('override_params_file'),
              {'use_sim_time': arg('use_sim_time')}]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=text)
          for name, default, text in ARGUMENTS],
        Node(package='scrubnurse_perception', executable='tool_detector', output='screen',
             parameters=params, remappings=[('image', arg('image_topic'))],
             condition=IfCondition(arg('detector'))),
        Node(package='scrubnurse_perception', executable='tool_localizer', output='screen',
             parameters=params, condition=IfCondition(arg('localizer'))),
        Node(package='scrubnurse_voice', executable='voice_command', output='screen',
             parameters=params,
             condition=IfCondition(arg('voice'))),
        Node(package='scrubnurse_dashboard', executable='dashboard', output='screen',
             parameters=[*params, {
                 'mode': arg('dashboard_mode'),
                 'host': arg('dashboard_host'),
                 'port': ParameterValue(arg('dashboard_port'), value_type=int),
             }],
             remappings=[('image', arg('image_topic'))],
             condition=IfCondition(arg('dashboard'))),
        Node(package='scrubnurse_control', executable='scrubnurse_controller', output='screen',
             parameters=[*params, {
                 'poses_file': ParameterValue(arg('poses_file'), value_type=str),
                 'home_pose': ParameterValue(arg('home_pose'), value_type=str),
                 'gripper.backend': arg('gripper_backend'),
                 'gripper.host': ParameterValue(arg('robot_ip'), value_type=str),
             }]),
        # Raw camera images would fill about 180 MB/s at 30 fps; record them as JPEG.
        Node(package='image_transport', executable='republish', arguments=['raw', 'compressed'],
             parameters=[{'out.jpeg_quality': 80}],
             remappings=[('in', arg('image_topic')),
                         ('out/compressed', RECORDED_CAMERA + '/compressed')],
             condition=IfCondition(arg('record'))),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o',
                 PathJoinSubstitution([arg('record_dir'), time.strftime('%Y-%m-%d_%H-%M-%S')]),
                 *RECORDED_TOPICS],
            output='screen', condition=IfCondition(arg('record'))),
    ])
