"""
Unified Entrance: Simulation（Gazebo + gazebo_ros2_control + controll_movej）Or real device（real_moveit）。

- mode:=sim / hardware_interface:=gazebo → controll_movej.launch.py（world → gazebo_world_file）
- mode:=real / hardware_interface:=real → real_moveit.launch.py（None Gazebo）
- enable_moveit:=true（Only allowed mode:=real）→ xMate_moveit_config.launch.py（move_group + RViz）
  Please use only under simulation first enable_gui / enable_movej；Simulation + MoveIt need to share the same URDF with Gazebo to avoid conflicts with xMate.urdf.xacro.

Example:
  ros2 launch rokae_hardware gazebo_moveit.launch.py \\
    mode:=sim robot_type:=CR35 world:=obstacles.world enable_gui:=true enable_movej:=false

  ros2 launch rokae_hardware gazebo_moveit.launch.py \\
    mode:=real robot_type:=CR35 robot_ip:=192.168.2.160 local_ip:=192.168.2.162 \\
    enable_moveit:=true use_sim_time:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def _resolve_mode(context, *args, **kwargs):
    mode_raw = LaunchConfiguration("mode").perform(context).strip().lower()
    hardware_raw = LaunchConfiguration("hardware_interface").perform(context).strip().lower()

    if mode_raw not in ("", "sim", "real"):
        raise RuntimeError(f"Invalid mode '{mode_raw}'. Expected 'sim' or 'real'.")
    if hardware_raw not in ("", "gazebo", "real"):
        raise RuntimeError(
            f"Invalid hardware_interface '{hardware_raw}'. Expected 'gazebo' or 'real'."
        )

    if hardware_raw:
        resolved_mode = "sim" if hardware_raw == "gazebo" else "real"
    else:
        resolved_mode = mode_raw or "sim"
    return [SetLaunchConfiguration("resolved_mode", resolved_mode)]


def _validate_args(context, *args, **kwargs):
    mode = LaunchConfiguration("resolved_mode").perform(context).strip().lower()
    if mode not in ("sim", "real"):
        raise RuntimeError(f"Invalid runtime mode '{mode}'. Expected 'sim' or 'real'.")

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
    )
    if mode == "real" and use_sim_time:
        raise RuntimeError("mode=real Need use_sim_time=false（Use the system clock on a real device）。")

    enable_gui = LaunchConfiguration("enable_gui").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
    )
    enable_movej = LaunchConfiguration("enable_movej").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
    )
    enable_moveit = LaunchConfiguration("enable_moveit").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
    )

    if mode == "sim" and enable_gui and enable_movej:
        raise RuntimeError(
            "mode=sim cannot be simultaneous enable_gui=true and enable_movej=true（Multiple sources competing for the same trajectory controller）。"
        )

    if mode == "real" and (enable_gui or enable_movej):
        raise RuntimeError(
            "mode=real Not supported enable_gui / enable_movej；Please use the plan enable_moveit:=true（MoveIt+RViz）Or externally send trajectory。"
        )

    if mode == "sim" and enable_moveit:
        raise RuntimeError(
            "mode=sim and enable_moveit Cannot be used simultaneously: Simulation model comes from xMate*_Gazebo.urdf.xacro，"
            "MoveIt Using xMate.urdf.xacro on the side will be duplicated/inconsistent. For simulation, please use it only. controll_movej of "
            "enable_gui or enable_movej；If you need MoveIt+Gazebo integration, please open a separate issue for dedicated integration. launch。"
        )

    if mode == "real":
        robot_ip = LaunchConfiguration("robot_ip").perform(context).strip()
        local_ip = LaunchConfiguration("local_ip").perform(context).strip()
        if not robot_ip or not local_ip:
            raise RuntimeError("mode=real Must be provided simultaneously robot_ip and local_ip。")
    return []


def _bringup(context, *args, **kwargs):
    hardware_pkg = get_package_share_directory("rokae_hardware")
    mode = LaunchConfiguration("resolved_mode").perform(context).strip().lower()

    sim_launch = os.path.join(hardware_pkg, "launch", "controll_movej.launch.py")
    real_launch = os.path.join(hardware_pkg, "launch", "real_moveit.launch.py")
    moveit_launch = os.path.join(hardware_pkg, "launch", "xMate_moveit_config.launch.py")

    robot_type = LaunchConfiguration("robot_type").perform(context)
    world = LaunchConfiguration("world").perform(context)
    enable_gui = LaunchConfiguration("enable_gui").perform(context)
    enable_movej = LaunchConfiguration("enable_movej").perform(context)
    enable_moveit_raw = LaunchConfiguration("enable_moveit").perform(context)
    enable_moveit_ui = enable_moveit_raw.strip().lower() in ("1", "true", "yes")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    local_ip = LaunchConfiguration("local_ip").perform(context)
    controller_manager_timeout = LaunchConfiguration("controller_manager_timeout").perform(context)
    service_call_timeout = LaunchConfiguration("service_call_timeout").perform(context)
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path").perform(context)

    actions = []

    if mode == "sim":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "robot_type": robot_type,
                    "gazebo_world_file": world,
                    "enable_gui": enable_gui,
                    "enable_movej": enable_movej,
                    "controller_manager_timeout": controller_manager_timeout,
                    "service_call_timeout": service_call_timeout,
                }.items(),
            )
        )
    else:
        # In the unified entry, MoveIt+RViz is provided by xMate_moveit_config Start; do not stack here real_moveit of movej
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(real_launch),
                launch_arguments={
                    "robot_type": robot_type,
                    "robot_ip": robot_ip,
                    "local_ip": local_ip,
                    "enable_moveit": "false",
                    "use_sim_time": use_sim_time,
                    "controller_manager_timeout": controller_manager_timeout,
                    "service_call_timeout": service_call_timeout,
                }.items(),
            )
        )

    if enable_moveit_ui:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments={
                    "robot_type": robot_type,
                    "robot_ip": robot_ip,
                    "local_ip": local_ip,
                    "use_fake_hardware": "false",
                    "use_sim_time": use_sim_time,
                    "warehouse_sqlite_path": warehouse_sqlite_path,
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_type",
                default_value="CR7",
                description="Model suffix (CR35, CR7, SR3, Pro3, etc.), and controll_movej / real_moveit consistent。",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="sim",
                description="sim or real; if set hardware_interface Then take it as the standard。",
            ),
            DeclareLaunchArgument(
                "hardware_interface",
                default_value="",
                description="Alias：gazebo → sim；real → real。Overwrite when not empty mode。",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Simulation true; for real machine please false。",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="empty.world",
                description="rokae_gazebo/worlds Download file name; only mode=sim Valid, incoming controll_movej of gazebo_world_file。",
            ),
            DeclareLaunchArgument("enable_gui", default_value="false"),
            DeclareLaunchArgument(
                "enable_movej",
                default_value="false",
                description="sim Enable the movej demonstration node below; do not use with enable_gui At the same time for true。",
            ),
            DeclareLaunchArgument(
                "enable_moveit",
                default_value="false",
                description="only mode=real：Start MoveIt move_group + RViz（scripture xMate_moveit_config.launch.py）。sim Do not turn on below。",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="",
                description="Real Machine Controller IP（mode=real Required）。",
            ),
            DeclareLaunchArgument(
                "local_ip",
                default_value="",
                description="This machine is located in the robot network segment IP（mode=real Required）。",
            ),
            DeclareLaunchArgument("warehouse_sqlite_path", default_value=""),
            DeclareLaunchArgument("controller_manager_timeout", default_value="120"),
            DeclareLaunchArgument("service_call_timeout", default_value="120"),
            OpaqueFunction(function=_resolve_mode),
            OpaqueFunction(function=_validate_args),
            OpaqueFunction(function=_bringup),
        ]
    )
