"""
Scrubnurse cell in Gazebo: UR10e with gripper, MoveIt 2 and the Scrubnurse nodes.

Tool detection is simulated from the true tool poses (sim_tool_detector).

The controller starts paused. Resume it and request a tool:

    ros2 service call /scrubnurse_controller/resume std_srvs/srv/Trigger
    ros2 topic pub --once /tool_request std_msgs/msg/String "{data: B}"

A tool (C) already lies in the return area, so the controller returns it to the
rack whenever no request is queued. A handed-over tool is taken by the simulated
surgeon (sim_surgeon), who lays it back on the table 15 s later for return. camera:=true adds the overhead camera
image (/overhead_camera/image); it is spawned once the controllers are up.
render_engine:=ogre2 looks better on native Linux; ogre (default) also works
under WSL.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

TOOLS = ['tool_A', 'tool_B', 'tool_C', 'tool_D', 'tool_E', 'tool_F', 'tool_C_table']
WORLD = 'scrubnurse_cell'


def launch_file(package, name):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), 'launch', name]))


def generate_launch_description():
    arg = LaunchConfiguration
    sim_time = {'use_sim_time': True}
    world_file = PathJoinSubstitution(
        [FindPackageShare('scrubnurse_sim'), 'worlds', arg('world')])
    bridged = [
        f'/world/{WORLD}/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/overhead_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
    ] + [f'/scrubnurse_sim/{tool}/{action}@std_msgs/msg/Empty]ignition.msgs.Empty'
         for tool in TOOLS for action in ('attach', 'detach')]
    tool_poses = (f'/world/{WORLD}/dynamic_pose/info', '/scrubnurse_sim/tool_poses')
    finger_spawner = Node(package='controller_manager', executable='spawner',
                          arguments=['finger_controller', '-c', '/controller_manager'])
    camera_spawner = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'overhead_camera', '-file', PathJoinSubstitution(
            [FindPackageShare('scrubnurse_sim'), 'models', 'overhead_camera.sdf'])])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=f'{WORLD}.sdf',
                              description='World file in scrubnurse_sim/worlds'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre',
                              description='Gazebo render engine: ogre or ogre2'),
        DeclareLaunchArgument('voice', default_value='false',
                              description='Start the voice command node (needs a microphone)'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='Spawn the overhead camera (only for viewing)'),
        IncludeLaunchDescription(
            launch_file('ur_simulation_gz', 'ur_sim_control.launch.py'),
            launch_arguments={
                'ur_type': 'ur10e',
                'description_package': 'scrubnurse_sim',
                'description_file': 'scrubnurse_cell.urdf.xacro',
                'runtime_config_package': 'scrubnurse_sim',
                'controllers_file': 'sim_controllers.yaml',
                'initial_joint_controller': 'joint_trajectory_controller',
                'world_file': ['--render-engine ', arg('render_engine'), ' ', world_file],
                'gazebo_gui': arg('gazebo_gui'),
                'launch_rviz': 'false',
            }.items()),
        IncludeLaunchDescription(
            launch_file('ur_moveit_config', 'ur_moveit.launch.py'),
            launch_arguments={
                'ur_type': 'ur10e',
                'use_sim_time': 'true',
                'launch_rviz': arg('launch_rviz'),
                'launch_servo': 'false',
            }.items()),
        finger_spawner,
        RegisterEventHandler(OnProcessExit(target_action=finger_spawner,
                                           on_exit=[camera_spawner]),
                             condition=IfCondition(arg('camera'))),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='scrubnurse_sim_bridge',
             arguments=bridged, remappings=[tool_poses], parameters=[sim_time]),
        Node(package='scrubnurse_sim', executable='sim_gripper', output='screen',
             parameters=[sim_time], remappings=[('tool_poses', tool_poses[1])]),
        Node(package='scrubnurse_sim', executable='sim_tool_detector', output='screen',
             parameters=[sim_time], remappings=[('tool_poses', tool_poses[1])]),
        Node(package='scrubnurse_sim', executable='sim_surgeon', output='screen',
             parameters=[sim_time], remappings=[('tool_poses', tool_poses[1])]),
        IncludeLaunchDescription(
            launch_file('scrubnurse_bringup', 'scrubnurse.launch.py'),
            launch_arguments={
                'override_params_file': PathJoinSubstitution(
                    [FindPackageShare('scrubnurse_bringup'), 'config', 'sim.yaml']),
                'use_sim_time': 'true',
                'detector': 'false',
                'gripper_backend': 'action',
                'home_pose': 'H',
                'voice': arg('voice'),
            }.items()),
    ])
