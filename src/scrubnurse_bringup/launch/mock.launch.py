"""
Motion check without hardware: UR10e on mock hardware, MoveIt 2 and the task controller.

The gripper is mocked as well.

No camera or microphone. Resume the controller and request a tool by hand:

    ros2 service call /scrubnurse_controller/resume std_srvs/srv/Trigger
    ros2 topic pub --once /tool_request std_msgs/msg/String "{data: B}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_file(package, name):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), 'launch', name]))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        IncludeLaunchDescription(launch_file('ur_robot_driver', 'ur_control.launch.py'),
                                 launch_arguments={
                                     'ur_type': 'ur10e',
                                     'robot_ip': '0.0.0.0',
                                     'use_fake_hardware': 'true',
                                     'launch_rviz': 'false',
                                     'launch_dashboard_client': 'false',
                                 }.items()),
        IncludeLaunchDescription(launch_file('ur_moveit_config', 'ur_moveit.launch.py'),
                                 launch_arguments={
                                     'ur_type': 'ur10e',
                                     'launch_rviz': LaunchConfiguration('launch_rviz'),
                                     'launch_servo': 'false',
                                 }.items()),
        IncludeLaunchDescription(launch_file('scrubnurse_bringup', 'scrubnurse.launch.py'),
                                 launch_arguments={
                                     'detector': 'false',
                                     'localizer': 'false',
                                     'voice': 'false',
                                     'gripper_backend': 'mock',
                                     'home_pose': 'H',
                                 }.items()),
    ])
