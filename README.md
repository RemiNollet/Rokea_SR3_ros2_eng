# Rokae ROS 2

Rokae ROS 2 software stack based on `ros2_control` and MoveIt 2. It provides simulation and real-robot control for the xMate series arms.

Current stack version: **0.0.4**

## Compatibility

| Item | Requirement |
|------|------|
| OS | Ubuntu 22.04 (recommended) |
| ROS 2 | Humble |
| xCore SDK | Match `rokae_hardware/sdk/VERSION` (currently **0.7.1**) |
| xCore controller | ≥ v3.2.1 |

## Repository contents

This repository contains ROS 2 source packages, URDF/MoveIt configuration, and examples. It does **not** include prebuilt xCore SDK libraries.

| Package | Description |
|---------|-------------|
| `rokae_hardware` | Hardware interface, `ros2_control` plugin, drivers, and SDK headers |
| `rokae_description` | URDF / meshes |
| `rokae_msgs` | Custom messages and services |
| `rokae_example` | Motion examples |
| `rokae_gazebo` | Gazebo simulation helpers |
| `rokae_xMate*_moveit_config` | MoveIt configuration per robot model |

## Get and build

### 1. Clone the repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your-github-repo-url> rokae_ros2
```

After cloning, the directory only needs to sit under `src/` for `colcon` to discover it.

### 2. Download prebuilt xCore SDK libraries

1. Check the SDK version in `rokae_hardware/sdk/VERSION`
2. Open the matching [xCoreSDK-CPP Release](https://github.com/RokaeRobot/xCoreSDK-CPP/releases) page
3. Download the library package for your platform (Linux example: `xCoreSDK-0.7.1-linux-x86_64.tar.gz`)
4. Extract it into `rokae_hardware/sdk/lib/` as described in [rokae_hardware/sdk/lib/README.md](rokae_hardware/sdk/lib/README.md)

### 3. Install ROS 2 dependencies and build

```bash
cd ~/ros2_ws
rosdep install --from-paths src/rokae_ros2 --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Documentation

- [User manual](doc/rokae%20ros2使用手册.md)
- [Demo launch notes](doc/README_demo.md)
- [Rokae online docs (ROS 2)](https://docs.rokae.com/docs/ROS2) — company documentation for ROS 2 and related topics

## Release notes

- Stack version: update `VERSION` in each package `package.xml` and `CMakeLists.txt`, and write `CHANGELOG.rst` / root `CHANGELOG.md`
- SDK upgrade: sync `rokae_hardware/sdk/include` headers, update `rokae_hardware/sdk/VERSION`, and note the required xCore SDK version in the GitHub Release notes

## License

Copyright (C) 2026 ROKAE (Beijing) Technology Co., LTD.

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE).
