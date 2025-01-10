from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler,IncludeLaunchDescription
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from scripts import GazeboRosPaths
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import sys

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from text_formatting import TextFormatting


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('rmcl_examples'), 'config')

    # Launch args
    map_arg = DeclareLaunchArgument('map', default_value=TextSubstitution(text="tray"),
                          description="map name. Choose between 'cube', 'cylinder', 'floor', 'sphere', 'tray', 'trays', 'avz'")

    map_name = LaunchConfiguration('map')
    prefix = LaunchConfiguration('prefix')

    map_world = TextFormatting('{}.world', map_name)

    # Needed for gazebo to find models
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }

    print("GAZEBO_MODEL_PATH", model_path)
    print("GAZEBO_PLUGIN_PATH", plugin_path)
    print("GAZEBO_RESOURCE_PATH", media_path)

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rmcl_examples"), "urdf", "robot.urdf.xacro"]
            ),
            " ",
            "name:=robot",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            # "gazebo_controllers:=",
            # config_husky_velocity_controller,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            'use_sim_time': True, 
            "publish_frequency": 100.0}, robot_description],
    )

    gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': PathJoinSubstitution([get_package_share_directory('rmcl_examples'), 'worlds', map_world]),
                'verbose': 'true',
                'params_file': os.path.join(config_dir, 'gazebo.yaml')
            }.items()
        )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=['-entity',
                   'robot',
                   '-topic',
                   'robot_description'],
        output='screen',
        parameters=[
                {'use_sim_time': True}]
    )

    imu_filter = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                os.path.join(config_dir, 'madgwick.yaml')],
        )

    ekf = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {"use_sim_time" : True}, 
                os.path.join(config_dir, 'ekf.yaml')],
           )

    ld = LaunchDescription([map_arg])
    ld.add_action(gzserver)
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(imu_filter)
    ld.add_action(ekf)

    return ld
