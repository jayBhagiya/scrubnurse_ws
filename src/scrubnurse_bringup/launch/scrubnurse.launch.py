"""
Scrubnurse application nodes: tool perception, voice commands and the task controller.

Expects the robot side (UR driver or simulation, plus MoveIt 2) and a camera
image topic to be up; real.launch.py, mock.launch.py and sim.launch.py include this file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    ('home_pose', '', 'Joint-space move to this pose before the first task (empty: off)'),
    ('use_sim_time', 'false', 'Use the /clock topic (simulation)'),
    ('detector', 'true', 'Start the YOLO tool detector'),
    ('localizer', 'true', 'Start the tool localizer (pixels to robot coordinates)'),
    ('voice', 'true', 'Start the voice command node'),
    ('voice_gui', 'true', 'Show the voice recording window'),
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
             parameters=[*params, {'use_gui': arg('voice_gui')}],
             condition=IfCondition(arg('voice'))),
        Node(package='scrubnurse_control', executable='scrubnurse_controller', output='screen',
             parameters=[*params, {
                 'poses_file': ParameterValue(arg('poses_file'), value_type=str),
                 'home_pose': ParameterValue(arg('home_pose'), value_type=str),
                 'gripper.backend': arg('gripper_backend'),
                 'gripper.host': ParameterValue(arg('robot_ip'), value_type=str),
             }]),
    ])
