from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler,IncludeLaunchDescription
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from scripts import GazeboRosPaths
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
ARGUMENTS = [
    DeclareLaunchArgument('map', default_value='',
                          description='The map name [tray, cube, ...]. defaults to tray.'),
]

def generate_launch_description():
    map_name = "tray.world"

    config_dir = os.path.join(get_package_share_directory('rmcl_example'), 'config')

    # Launch args
    map_name = LaunchConfiguration('map')
    prefix = LaunchConfiguration('prefix')

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
                [FindPackageShare("rmcl_example"), "urdf", "robot.urdf.xacro"]
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
                'world': os.path.join(get_package_share_directory('rmcl_example'), 'worlds', map_name + ".world"),
                'verbose': 'true',
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

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gzserver)
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(imu_filter)
    ld.add_action(ekf)

    return ld
