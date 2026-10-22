"""
Real Scrubnurse cell: UR10e driver, MoveIt 2, the overhead Azure Kinect and the Scrubnurse nodes.

After launching, start the External Control program on the pendant.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_file(package, name):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), 'launch', name]))


def config(name):
    return PathJoinSubstitution([FindPackageShare('scrubnurse_bringup'), 'config', name])


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='IP address of the UR controller'),
        DeclareLaunchArgument(
            'kinematics_params_file', default_value=config('ur10e_calibration.yaml'),
            description='Factory calibration of this UR10e (ur_calibration format)'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Start the Azure Kinect driver'),
        DeclareLaunchArgument('image_topic', default_value='/od_camera/stream/rgb'),
        DeclareLaunchArgument('voice_gui', default_value='true'),
        IncludeLaunchDescription(launch_file('ur_robot_driver', 'ur_control.launch.py'),
                                 launch_arguments={
                                     'ur_type': 'ur10e',
                                     'robot_ip': arg('robot_ip'),
                                     'kinematics_params_file': arg('kinematics_params_file'),
                                     'launch_rviz': 'false',
                                 }.items()),
        IncludeLaunchDescription(launch_file('ur_moveit_config', 'ur_moveit.launch.py'),
                                 launch_arguments={
                                     'ur_type': 'ur10e',
                                     'launch_rviz': arg('launch_rviz'),
                                     'launch_servo': 'false',
                                 }.items()),
        Node(package='scrubnurse_perception', executable='kinect_camera', output='screen',
             parameters=[config('scrubnurse.yaml'), {
                 'camera_info_file': config('azure_kinect_rgb_1080P.yaml')}],
             remappings=[('image', arg('image_topic')),
                         ('depth', '/od_camera/stream/depth'),
                         ('camera_info', '/od_camera/camera_info')],
             condition=IfCondition(arg('camera'))),
        IncludeLaunchDescription(launch_file('scrubnurse_bringup', 'scrubnurse.launch.py'),
                                 launch_arguments={
                                     'image_topic': arg('image_topic'),
                                     'voice_gui': arg('voice_gui'),
                                     'robot_ip': arg('robot_ip'),
                                 }.items()),
    ])
