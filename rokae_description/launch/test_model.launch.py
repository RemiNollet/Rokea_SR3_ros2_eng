"""
Minimize Gazebo Verification (Phase 1.3): Launch Simulation Only、robot_state_publisher、spawn Model.
Not included ros2_control / Joint GUI. Used to check whether the mesh and geometry are normal.。
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
import xacro


def _setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    pkg_desc = get_package_share_directory('rokae_description')
    pkg_gz = get_package_share_directory('rokae_gazebo')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'xMateCR7_Gazebo.urdf.xacro')
    mesh_prefix = (Path(pkg_desc) / 'meshes').as_uri()
    robot_desc = xacro.process_file(
        xacro_file, mappings={'mesh_prefix': mesh_prefix}
    ).toxml()

    world_path = world
    if not os.path.isabs(world_path):
        world_path = os.path.join(pkg_gz, 'worlds', world_path)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            )
        ),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--verbose',
        }.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True},
        ],
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'xMateCR7',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
        output='screen',
    )

    return [gz, rsp, spawn]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty.world',
            description='rokae_gazebo/worlds Enter the file name or absolute path',
        ),
        OpaqueFunction(function=_setup),
    ])
