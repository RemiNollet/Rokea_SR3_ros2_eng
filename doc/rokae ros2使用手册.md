# Rokae  ROS2Instruction Manual

## Introduction

### ROS2Introduction

ROS2 (Robot Operating System 2) is a comprehensive upgrade of the Robot Operating System (ROS), designed to meet industrial-level requirements for real-time performance, security, and cross-platform compatibility.  
ROS2 is an upgraded version of ROS1, but there are some differences:

- ROS2 adopts a decentralized architecture, allowing nodes to discover and communicate directly
- Introduces a standardized, managed node lifecycle, supporting hard real-time for higher precision robots
- Uses a more efficient ament build system and employs colcon as the new build tool  

### Purpose and Scope  

**Purpose**  
This document provides a guide for installing and configuring the Rokae ROS2 package, and includes tutorials for both simulation and real robot operations.  
This manual is intended for developers who are familiar with basic ROS2 concepts and wish to integrate Rokae robots into their applications.。  
**Scope**  
The current version of the Rokae ROS2 package supports xMate series CR7, CR12, CR18, CR20, CR35, ER3, ER7, Pro3, Pro7, SR3, SR4, SR5, and AR model robotic arms. More models will be supported in future updates, and users can also customize adaptations for new models according to their needs.

### Online Documentation

In addition to this manual, you can learn about ROS 2 packages and related content through the official Luoshi online documentation.：

- [ROS2 Software Package Documentation (Online）](https://docs.rokae.com/docs/ROS2)

### Currently Compatible Models

AR Series: xMateAR5L, xMateAR5R

CR Series: xMateCR7, xMateCR12, xMateCR18, xMateCR20, xMateCR35

ER Series: xMateER3, xMateER7

Pro Series: xMatePro3, xMatePro7

SR Series:xMateSR3,xMateSR4,xMateSR5



## Install

### Environment Configuration

Rokae ROS2 is mainly developed and tested on Ubuntu 22.04 + ROS2 Humble, and there may be incompatibilities in other environments.

- Hardware Requirements

    |Component   |Configuration Requirements   |Remarks   |
    | ------------ | ------------ | ------------ |
    |CPU   |64 Intel i5/i7 (or equivalent AMD processor)）   |It is recommended to use an 8-core or higher processor to ensure 1000Hz real-time mode.   |
    | RAM  |8Gor16G   | MoveIt 2 Motion planning and RViz visualization require a large amount of memory  |
    |Storage space   |20GThe above   | Faster storage can improve boot speed and data processing capability  |
    |GPU   |CUDA-supported NVIDIA GPU   |Not necessary   |

- ROS2 Environment Setup (humble)  
One-click installation of Fish-Flavored ROS2: https://blog.csdn.net/pixelprodigy/article/details/147933853  
Installation of moveit, controller-manager, and related packages  

    ```bash
        sudo apt update 
        sudo apt install ros-humble-moveit
        sudo apt install ros-humble-controller-manager
        sudo apt install ros-humble-joint-state-broadcaster 
        sudo apt install \
            ros-humble-joint-state-publisher\
            ros-humble-forward-command-controller \
            ros-humble-effort-controllers \
            ros-humble-velocity-controllers \
            ros-humble-position-controllers \
            ros-humble-joint-trajectory-controller
        ##If there are any other packages not found, you can install them yourself using sudo apt install
        source /opt/ros/humble/setup.bash
    ```

- Create a local workspace
Clone this repository from GitHub to the workspace `src` Table of contents, and according to [README.md](../README.md) Download xCore SDK precompiled library to `rokae_hardware/sdk/lib/`。

    ```bash
        ## Create ros2_ws The workspace must contain subdirectories src
        mkdir -p ~/ros2_ws/src
        cd ~/ros2_ws/src
        git clone <your-github-repo-url> rokae_ros2
        ## press rokae_hardware/sdk/lib/README.md Download and place the xCore SDK library
        cd ~/ros2_ws
        colcon build --symlink-install
    ```
    ！！It is recommended to refresh the environment variables -- in **.bashrc**Add the following content at the end of the file and save  
    !! at **home**Enter shortcut key under the folder**ctrl+h**Show hidden files **.bashrc**
    ```bash
        source /opt/ros/humble/setup.bash
        source ~/ros2_ws/install/local_setup.sh
        source ~/ros2_ws/install/setup.bash
    ```

## Workspace Overview

### rokaePackage Overview

​    ├── doc------------Manual Contents  
​    ├── rokae_description------------Store URDF and configuration files that describe the robot model
​    ├── rokae_hardware------------The main folder has the following specific structure 
​    ├── rokae_msgs------------Custom messages used in other packages 
​    ├── rokae_xMateAR5L_moveit_config------------of each modelmoveit_configConfiguration File  
​    ├── rokae_xMateAR5R_moveit_config
​    ├── rokae_xMateCR7_moveit_config
​    ├── rokae_xMateCR12_moveit_config
​    ├── rokae_xMateCR18_moveit_config
​    ├── rokae_xMateCR20_moveit_config
​    ├── rokae_xMateCR35_moveit_config
​    ├── rokae_xMateER3_moveit_config
​    ├── rokae_xMateER7_moveit_config
​    ├── rokae_xMatePro3_moveit_config
​    ├── rokae_xMatePro7_moveit_config
​    ├── rokae_xMateSR3_moveit_config
​    ├── rokae_xMateSR4_moveit_config
​    └── rokae_xMateSR5_moveit_config

### rokae_hardwarePackage structure

​    ├── CMakeLists.txt  
​    ├── config------------Controller configuration file  
​    ├── include------------Hardware interface header file  
​    ├── launch------------Files to start each node  
​    ├── package.xml  
​    ├── rokae_hardware_interface.xml
​    ------------ROS2 Control Plugin description file in the framework, used to register hardware interfaces with the ROS2 control system  
​    ├── sdk------------sdkRelated package  
​    └── src------------Specific implementation in C++, the specific files are as follows  

### srcFile Structure

​    ├── connect_test.cpp------------Network performance analysis test!! Nodes not written, after compiling look for the binary executable in the build directory to run
​    ├── movej_client.cpp------------movejFunction client example, withrokae_driver(Server side)Use together 
​    ├── movej_moveit_test.cpp------------Implementation of movej based on MoveIt (MoveIt planning)） 
​    ├── rokae_driver.cpp------------6Axis robot encapsulates specific interfaces（ros2 service）
​    ├── rokae_driver7.cpp------------7Axis robot encapsulates specific interfaces（ros2 service）
​    └── rokae_hardware_interface.cpp------------Specific implementation of hardware interface


## Beginner's Guide

### ROS2_controlArchitecture

rokae uses ros2ros2_controlArchitecture
Reference Learning Websites：https://control.ros.org/rolling/doc/getting_started/getting_started.html  

Under this architecture, only a few key files need to be provided

- Controller-related: This part is stored in a yaml file, generally containing parameters related to the actual control algorithm, mainly located at**rokae_hardware/config**Under the directory
- Hardware-related parameters: This part is saved in the URDF, mainly located at**rokae_deacription/urdf**Under the directory
- Hardware interface implementation: located at**rokae_hardware/src/rokae_hardware_interface.cpp**Internally called rokae's API to implement control of the robot, with the hardware interface as**plugin**in the form used by ROS2controller_managerLoad and Manage
  

### Using MoveIt in RViz to plan a real robotic arm

(1) Ensure MoveIt 2、rokae_ros2、ros2_control The package has been installed correctly  
(2)Execute the launch file

```bash
    ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=SR4 use_fake_hardware:=true robot_ip:=192.168.2.160 local_ip:=192.168.2.1
```
What does 'rt' mean here?
！！！**Attention**！！！  
in the example `robot_type` Change to your model&emsp;&emsp;For example, AR5L, AR5R, SR4, ER7, Pro3, Pro7, CR7, CR12, CR18, CR20, CR35, etc.  
**robot_ip**Corresponding robotip       **local_ip**Corresponding to this deviceip  
**use_fake_hardware**:When using a virtual hardware interface to connect to a physical robot or HMI, set to**false**，It is recommended to use first`use_fake_hardware = true`Virtual hardware interface testing
When connecting to a physical robot, pay attention to the RCI settings in the HMI and the packet loss rate
Changing the model (six-axis/seven-axis): in**rokae_hardware_interface.cpp**Lines 227-228 and**rokae_hardware_interface.h**Uncomment the corresponding lines in lines 95-96 of the header file, the code is as follows  

```cpp
    /*rokae_hardware_interface.cpp*/
    robot_ = std::make_shared<rokae::xMateRobot>(robot_ip_, local_ip_);   //Even six-axis models
    // robot_ = std::make_shared<rokae::xMateErProRobot>(robot_ip_, local_ip_);     //Including seven-axis models
    //Depending on the number of axes of the model, shared pointers need to be adjustedrobot_Modify the definition and initialization。


    /*rokae_hardware_interface.h*/
    std::shared_ptr<rokae::xMateRobot> robot_;     //Even six-axis models
    // std::shared_ptr<rokae::xMateErProRobot> robot_;    //Seven-axis machine model
```

(3)Path planning in rviz:

- The yellow robotic arm model represents the goal position, the white solid model represents the real robotic arm model, and the gray transparent robotic arm shows the initial position.
- Click the interactive marker (represented as a sphere at the robot's end effector) and move it to the desired target position, or under MotionPlanning**Joints**Modify the joint angles of the goal position.
- Click "Plan & Execute" Generate and visualize the robot's trajectory, where you can see the white robotic arm moving to the pose of the yellow robotic arm. 
- When planning movements multiple times consecutively, it is recommended to first click the interactive sphere at the end of the robotic arm in RViz to update the joint information under Joints in MotionPlanning, which helps with the next movement planning.
- To modify the movement speed: on the right side of Planning under MotionPlanning.Options,**VelocityScaling**，ratio is0-1。

![hmiStatus Monitoring](image.png) 
Figure 1 HMI Status Monitoring 
![rvizVisualization](image-1.png)  
Figure 2 rviz visualization  
![MotionpPlanningofJoints](image-2.png)  
Figure 3 Modify the target position according to the Joints in MotionPlanning  
![Modify target location](image-3.png)  
rvizTarget attitude and actual attitude  

(4)Usemovej_moveit_test.cppControl the robotic arm  
This is a movej implementation based on MoveIt planning  
Start it while ensuring the above launch runs normallycontroll_movej.launch.py

```bash
    ros2 launch rokae_hardware controll_movej.launch.py robot_type:=SR4
```

！！！**movej.cppPay attention to the following changes**！！！

```cpp
    
    /*149OK    Replace the base with the corresponding model(Under the corresponding model SRDF)*/
    arm.setPoseReferenceFrame("AR5-5_07R-W4C4A2_base");    //xxx_base  Suggested changes (comments available）

    /*158Row   Target Joint Angle(Pay attention to the number of axles)*/
    std::vector<double> joint_target = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}; 

    /*284Okay, replace the corresponding oneplanning group(Under the corresponding model SRDF)*/
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "AR5R_arm");
    //xmateSeries defaults torokae_arm 
```

#### Main Nodes

- Use on the terminal`ros2 node list`Query
- Use to check specific information for each node`ros2 node info /<node_name>`&emsp;Display topic/service/action communication under the node

    | Node Name | Description |
    |----------|------|
    | /controller_manager | Controller nodes, included with ROS2, implement hardware interfaces in the form of plugins, and are controller_manager Dynamic loading |
    | /interactive_marker_display_100663865840352 | Rviz Interactive Markup Display |
    | /joint_state_broadcaster | Joint state broadcaster, reads the actual positions, velocities, and torques of the robot's joints, and publishes to `/joint_states` Topic |
    | /joint_state_publisher | Joint State Publisher (used when the robot has no hardware interface)） |
    | /move_group | MoveIt2 core planning nodes |
    | /move_group_private_105329056153616 | MoveIt2 internal private node |
    | /moveit_simple_controller_manager | Connect the MoveIt planner with the actual robot controller |
    | /position_joint_trajectory_controller | Joint position trajectory controller, receives the joint trajectory planned by MoveIt and sends it to the robot hardware interface |
    | /robot_state_publisher | Robot state publisher, subscribe `/joint_states` Topic, calculated based on the robot URDF TF |
    | /rviz2 | Rviz2 Node |
    | /rviz2_private_126572619198304 | Rviz2 Internal private node |
    | /transform_listener_impl_5b8da1b44ef0 | TF Transform Listener Implementation |
    | /transform_listener_impl_5fcbd6902460 | TF Transform Listener Implementation |
    | /transform_listener_impl_731dfddfc690 | TF Transform Listener Implementation |

#### Main topic

- Using in the terminal`ros2 topic list`Query  
- Use for specific information on each topic`ros2 topic info /<topic_name>`&emsp;Display message type/publisher and subscriber count
- Use`ros2 topic echo /<topic_name>`Output message  

    | topicName | Description |
    |----------|------|
    | /joint_states | Real-time status of all the robot's joints |
    | /display_planned_path | Information on the interpolated points of the planned trajectory |
    | /position_joint_trajectory_controller/controller_state | Controller Status |
    | /position_joint_trajectory_controller/joint_trajectory | Send trajectory command to the controller (not used, using action communication)） |


#### Main Action

- Use in the terminal`ros2 action list`Query
- Use to check specific information for each action`ros2 action info /<action_name>`Query

    | actionName | Description |
    |--------------|------|
    | /execute_trajectory | Execution path, return success or failure |
    | /move_action | Plan and execute the path, return success or failure |
    | /position_joint_trajectory_controller/follow_joint_trajectory | Communicate with MoveIt to send the controller to execute the trajectory |


### rokae_driver Drive the robot  

rokae_driverThe package is responsible for low-level robot communication and control, using ros2 service and topic communication, and encapsulates somexCore API

#### Packaged Functions

- service
Usage: ros2 service call / write a client
Service messages are located at/rokae_msgs/srv  

    | Service/Function Name | Description | Message Type |
    |--------------|------|----------|
    | get_robot_info | Obtain basic information about the robot, such as model and SDK version | `GetRobotInfo` |
    | jog_control | JogMode Control | `JogCon` |
    | drag_control | Drag mode on/off | `DragCon` |
    | calculate_fk | Calculate Forward Kinematics (Forward Solution） | `CalculateFK` |
    | calculate_ik | Calculating Inverse Kinematics (Inverse Solution） | `CalculateIK` |
    | get_di | Read DI (Digital Input) signal | `GetDI` |
    | set_di | Set DI (Digital Input) signal | `SetDI` |
    | get_do | Read DO (Digital Output) signal | `GetDO` |
    | set_do | Set DO (Digital Output) signal | `SetDO` |
    | movej | MoveJReal-time joint movement | `MoveJ` |
    | movel | MoveLLinear real-time motion | `MoveL` |
    | movec | MoveCArc real-time motion | `MoveC` |
    | read_register | Read register | `ReadRegister` |
    | write_register | Write register | `WriteRegister` |  

- topic  
  Usage: ros2 topic echo/subscriber listen

    | Topic Name | Description | Message Type |
    |----------|------|----------|
    | /rokae_driver/joint_states | Publish robot joint angle status | `sensor_msgs/msg/JointState` |
    | /rokae_driver/cartesian_pose | Publish robot Cartesian space pose | `geometry_msgs/msg/PoseStamped` |

#### Startup method

```bash
    ros2 run rokae_hardware rokae_driver --ros-args -p robot_ip:=192.168.2.160 -p local_ip:=192.168.2.100
    ros2 launch rokae_hardware rokae_driver.launch.py robot_ip:=192.168.2.160 local_ip:=192.168.2.100
    #launchYou can choose either 'or' or 'run', just pay attention to the IP address.

    ##Example：get_robot_info
    ros2 service call /rokae_driver/get_robot_info rokae_msgs/srv/GetRobotInfo
```

### Adapt to new models

- **robot_description**Import the corresponding rviz, mesh, and xacro files, and the file formats can follow the existing model.  
- **robot_description** of**urdf**in the documentxMate.urdf.xacro,xMate_macro.xacroAdd the corresponding model configuration, and create a new modelros2_controll.xacroDocument, the specific format can refer to the existing document models
- Use**moveit_setup_assistant**Configure the corresponding modelmoveit_configFolder, where there are differences, the existing model format shall prevail

---

## Gazebo Simulation and Trajectory Development

### Explanation

- In the example below, the workspace path is `~/ros2_ws`、Source code directory starts with `~/ros2_ws/src/rokae_ros2` As an example, please replace it with the actual path of this machine; it is recommended that each new terminal executes it first：  
  `source ~/ros2_ws/install/setup.bash`
- Joint Naming: Instructions, Trajectories, and Controller Configurations are unified as in the documentation and examples `joint1`～`joint6`（Six-axis) or `joint1`～`joint7`（Seven axes); the soft limits of each axis, singularities, and safety space still follow `rokae_description/urdf/` Corresponding models (including `*_Gazebo*.urdf.xacro`）of `<limit lower upper>` As the standard; be sure before issuing the trajectory `ros2 topic echo /joint_states` Verify the number of axes and joint names。

### 1. Environment and Scene Support

#### 1.1 One-click load of obstacle scenes

Parameters `gazebo_world_file:=obstacles.world` when, by `test_model.launch.py` Load `rokae_gazebo/worlds/obstacles.world`（Obstacle space). The aircraft model and scenario are decoupled, only modified `robot_type` That's it.

Terminal 1 — Load startup instructions

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch rokae_hardware test_model.launch.py robot_type:=Pro3 gazebo_world_file:=obstacles.world gui:=true
```

Supported models (such as `SR3`、`SR4`、`SR5`、`CR7`、`CR12`、`CR18`、`CR20`、`CR35`、`ER3`、`ER7`、`Pro3`、`Pro7`、`AR5L`、`AR5R` etc.) similarly replace `robot_type`。Default empty world is `empty.world`；Do not add `gazebo_world_file` The empty world。

#### 1.2 Scene Loading and Saving Service
Node `scene_service`（bag `rokae_hardware`）Provide：

| Service Name | Function |
|--------|------|
| `/load_scene` | Search by scene name `.world`，Verify the path and register it as a parameter for reference by other modules。 |
| `/save_scene` | Copy the template world to the user directory and save a custom scene snapshot。 |

Service Type：`rokae_hardware/srv/SceneService`，Field `scene_name`（String).

Terminal 1 — Start simulation with obstacles (example SR3）

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch rokae_hardware test_model.launch.py robot_type:=SR3 gazebo_world_file:=obstacles.world gui:=true
```

Keep running.

Terminal 2 — Start the scene service

```bash
source ~/ros2_ws/install/setup.bash

ros2 run rokae_hardware scene_service
```

A typical log line is similar：

```text
[INFO] [scene_manager]: Scene Service: /load_scene /save_scene (world bag: rokae_gazebo)
```

Terminal 3 — Call `/load_scene`（Must be executed after Terminal 2 is running）

```bash
source ~/ros2_ws/install/setup.bash

ros2 service call /load_scene rokae_hardware/srv/SceneService "{scene_name: 'obstacles.world'}"
```

Upon success `success: true`，`message` It will provide the parsed absolute path of the world; and it will prompt: if you need to replace a Gazebo world that is already running, you should first close Gazebo, and then start the file using the world parameter of launch.

Terminal 3 — Invocation `/save_scene` Save Snapshot Example

```bash
ros2 service call /save_scene rokae_hardware/srv/SceneService "{scene_name: 'my_cr7_snapshot'}"
```

Upon success `message` It will give a save path, generally under the user's directory. `~/.rokae_gazebo/saved_worlds/<Name>.world`。

#### 1.3 Collision Detection Verification

Obstacle World and `arm_controller` Available after running `rokae_gazebo` in the bag `collision_test.launch.py` Subscribe `ContactsState`（Default `/obstacle/bumper_contact`），Check the collision pairs in the terminal log.

Terminal 1 — Simulation (example SR5, with Terminal 2's `robot_type` Must be consistent）

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch rokae_hardware test_model.launch.py robot_type:=SR5 gazebo_world_file:=obstacles.world gui:=true
```

Terminal 2 — Collision Monitoring

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch rokae_gazebo collision_test.launch.py robot_type:=SR5 publish_motion:=false contact_topic:=/obstacle/bumper_contact
```

- `publish_motion:=false`：Do not automatically send trajectory within this node, rely on the slider of Terminal 1 or external trajectory top obstacle.
- Default press `robot_collision_substring:=xMate` Filter out contacts unrelated to the robotic arm (can be adjusted in the launch parameters). 

After startup, terminal 2 may show something similar：

```text
[INFO] [collision_test_node]: Collision Listener: /obstacle/bumper_contact；Trajectory topic: /arm_controller/joint_trajectory（Please start arm_controller）
```

In the event of a collision, it may appear in the logs：

```text
[WARN] [collision_test_node]: New collision contact detected ... Collider: [table::link::collision] <-> [xMateSR5::xMateSR5_link2::xMateSR5_link2_collision]
```

### Two、ros2_control Integration and Trajectory Control

#### 2.1 Simulation Control Closed Loop

- Gazebo Plugin `libgazebo_ros2_control.so` Start `controller_manager`，and ROS 2 `ros2_control` Stack docking。
- `test_model.launch.py` Pass after delay `spawner` Load in sequence：
  - `joint_state_broadcaster`：Read the simulated joint state and publish `/joint_states`；
  - `arm_controller`（`JointTrajectoryController`）：Subscribe `/arm_controller/joint_trajectory`，will `trajectory_msgs/JointTrajectory` Switch to joint position command-driven simulation.
- Link: ROS 2 trajectory topic → Trajectory Controller → Gazebo Joint。

#### 2.2 controll_movej.launch.py

Load the aircraft model in Gazebo URDF、`gazebo_ros2_control`、`joint_state_broadcaster` and `arm_controller`；Relative `test_model.launch.py` Can also be integrated with MoveIt configuration and is optional `movej` Demo node.

Important: Do not do it at the same time `enable_gui:=true` and `enable_movej:=true`，Avoid multiple sources competing for the same trajectory controller.

Mode A — GUI slider (terminal 1）

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch rokae_hardware controll_movej.launch.py robot_type:=SR3 enable_gui:=true enable_movej:=false
```

approximately 5～6 Drag again after a few seconds `joint_state_publisher_gui` Slide bar; slide bar sleeve `gui_to_joint_trajectory` Form into a short-term trajectory and send to `/arm_controller/joint_trajectory`。Close `joint_state_publisher_gui` After the interface, you can also directly switch to mode B.

Mode B — movej demonstration + terminal trajectory sending (terminal 1）

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch rokae_hardware controll_movej.launch.py robot_type:=SR3 enable_gui:=false enable_movej:=true
```

Open another terminal and send `ros2 topic pub ...`（see 2.4）。

#### 2.3 Verify /joint_states

When verifying, ensure that Terminal 1 has started the Gazebo scene and that the model has been successfully loaded and displayed。

```bash
source ~/ros2_ws/install/setup.bash

ros2 topic echo /joint_states --once
```

Under a unified name，`name` The field should be `joint1`…`joint6` or contain `joint7`（seven-axis), with `rokae_hardware/config/xMate{Aircraft model}_controllers.yaml` Central controller `joints` List consistent。

#### 2.4 Trajectory topics and ros2 topic pub examples

(1) Confirm who is currently subscribing to the trajectory

```bash
source ~/ros2_ws/install/setup.bash

ros2 control list_controllers

ros2 topic list | grep joint_trajectory
```

- Gazebo `test_model` / `controll_movej` In this scenario, it is commonly `/arm_controller/joint_trajectory`。
- If only `position_joint_trajectory_controller` For active (for example, used alone `rokae_moveit_launch.py` when the controller is pulled up by default), it should be directed to `/position_joint_trajectory_controller/joint_trajectory` Release.

(2) Six-axis — Terminal 1

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch rokae_hardware controll_movej.launch.py robot_type:=SR3 enable_gui:=false enable_movej:=false
```

Terminal 2 — Two-stage Waypoints (Unit rad）

```bash
source ~/ros2_ws/install/setup.bash

ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
points:
  - positions: [0.20, -0.20, 0.15, 0.00, 0.20, 0.00]
    time_from_start: {sec: 3, nanosec: 0}
  - positions: [0.20, -0.55, 0.25, 0.10, 0.30, 0.10]
    time_from_start: {sec: 6, nanosec: 0}
"
```

（3）Seven-axis — `joint_names` With each `positions` Must be 7-dimensional

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
points:
  - positions: [0.10, -0.20, 0.15, 0.00, 0.20, 0.00, 0.00]
    time_from_start: {sec: 3, nanosec: 0}
  - positions: [0.20, -0.35, 0.25, 0.10, 0.30, 0.10, 0.00]
    time_from_start: {sec: 6, nanosec: 0}
"
```

（4）Reference values for joint angle soft limits of each model (radians)  
The values in the table below come from the warehouse Gazebo xacro `<limit>`，For preliminary check before issuing commands only; after modifying the URDF, the file shall prevail. The axis names are standardized as `joint1`～`joint6`（and `joint7`） Order corresponds to machine number 1～7 Axis。

| Aircraft model | joint1 | joint2 | joint3 | joint4 | joint5 | joint6 | joint7 |
|------|--------|--------|--------|--------|--------|--------|--------|
| SR3 | [-3.0543, 3.0543] | [-2.3562, 2.2689] | [-3.0543, 2.3562] | [-3.0543, 3.0543] | [-3.0543, 3.0543] | [-3.0543, 3.0543] | — |
| SR4 | [-3.0543, 3.0543] | [-2.3562, 2.3562] | [-2.3562, 2.3562] | [-3.0543, 3.0543] | [-3.0543, 3.0543] | [-3.0543, 3.0543] | — |
| SR5 | [-6.2832, 6.2832] | [-2.7925, 2.618] | [-2.9671, 2.4435] | [-6.2832, 6.2832] | [-6.2832, 6.2832] | [-6.2832, 6.2832] | — |
| CR7 | [-3.0543, 3.0543] | [-3.0543, 3.0543] | [-3.0543, 3.0543] | [-3.0543, 3.0543] | [-3.0543, 3.0543] | [-3.0543, 3.0543] | — |
| CR12 | [-6.2832, 6.2832] | [-2.9671, 2.9671] | [-6.2832, 6.2832] | [-6.2832, 6.2832] | [-6.2832, 6.2832] | [-6.2832, 6.2832] | — |
| CR18 | [-3.0543, 3.0543] | [-2.9671, 2.9671] | [-2.8798, 2.8798] | [-3.0543, 3.0543] | [-3.0543, 3.0543] | [-3.0543, 3.0543] | — |
| CR20 | [-3.0543, 3.0543] | [-3.0543, 3.0543] | [-2.9671, 2.9671] | [-3.0543, 3.0543] | [-3.0543, 3.0543] | [-3.0543, 3.0543] | — |
| CR35 | [-6.2832, 6.2832] | [-6.2832, 6.2832] | [-2.8972, 2.8972] | [-6.2832, 6.2832] | [-6.2832, 6.2832] | [-6.2832, 6.2832] | — |
| ER3 | [-2.9671, 2.9671] | [-2.0944, 2.0944] | [-2.0944, 2.0944] | [-2.9671, 2.9671] | [-2.0944, 2.0944] | [-6.2832, 6.2832] | — |
| ER7 | [-2.9671, 2.9671] | [-2.0944, 2.0944] | [-2.0944, 2.0944] | [-2.9671, 2.9671] | [-2.0944, 2.0944] | [-6.2832, 6.2832] | — |
| Pro3 / Pro7 | [-2.9671, 2.9671] | [-2.0944, 2.0944] | [-2.9671, 2.9671] | [-2.0944, 2.0944] | [-2.9671, 2.9671] | [-2.0944, 2.0944] | [-6.2832, 6.2832] |
| AR5L / AR5R | [-3.1067, 3.1067] | [-2.0944, 2.0944] | [-3.1067, 3.1067] | [-1.0472, 2.5307] | [-3.1067, 3.1067] | [-1.0472, 1.0472] | [-1.0472, 1.0472] |

### 3. One-click launch (Gazebo simulation / real machine command drive)）

#### 3.1 Entrance and internal links

```bash
ros2 launch rokae_hardware gazebo_moveit.launch.py ...
```

| Condition | child launch | Function |
|------|-----------|------|
| `mode:=sim` | `controll_movej.launch.py` | Gazebo + `gazebo_ros2_control`；`world` → child launch `gazebo_world_file`；Optional `enable_gui` / `enable_movej`。 |
| `mode:=real` | `real_moveit.launch.py` | This device `ros2_control_node` + Real device IP, load `xMate{Aircraft model}_real_controllers.yaml`；Does not start Gazebo。 |

#### 3.2 Parameter constraints

1. `mode:=real` time `use_sim_time:=false`（Use the system clock on a real device）。
2. `mode:=sim` Cannot happen at the same time `enable_gui:=true` and `enable_movej:=true`。
3. `mode:=real` Must be provided simultaneously `robot_ip` and `local_ip`。

#### 3.3 Simulation Mode Example

Empty World + Slider（SR3）

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch rokae_hardware gazebo_moveit.launch.py mode:=sim robot_type:=SR3 world:=empty.world enable_gui:=true enable_movej:=false
```

Obstacle World（CR7）

```bash
ros2 launch rokae_hardware gazebo_moveit.launch.py mode:=sim robot_type:=CR7 world:=obstacles.world enable_gui:=true enable_movej:=false
```

Obstacle World（CR35）

```bash
ros2 launch rokae_hardware gazebo_moveit.launch.py mode:=sim robot_type:=CR35 world:=obstacles.world enable_gui:=true enable_movej:=false
```

Open `movej` Demo, Off GUI

```bash
ros2 launch rokae_hardware gazebo_moveit.launch.py mode:=sim robot_type:=CR7 world:=empty.world enable_gui:=false enable_movej:=true
```

#### 3.4 Real Device Mode Example

Terminal 1 — Start the real device control stack (none Gazebo）

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch rokae_hardware gazebo_moveit.launch.py mode:=real robot_type:=CR7 robot_ip:=192.168.2.160 local_ip:=192.168.2.162 enable_moveit:=false use_sim_time:=false
```

Terminal 1 — Six-axis CR35 real machine (only need to `robot_type` Change IP to local value）

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch rokae_hardware gazebo_moveit.launch.py mode:=real robot_type:=CR35 robot_ip:=192.168.2.160 local_ip:=192.168.2.162 enable_moveit:=false use_sim_time:=false
```

Terminal 2 — Output Joint Trajectory (Six-Axis)）

```bash
source ~/ros2_ws/install/setup.bash

ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
points:
  - positions: [0.10, -0.20, 0.15, 0.00, 0.20, 0.00]
    time_from_start: {sec: 3, nanosec: 0}
  - positions: [0.20, -0.35, 0.25, 0.10, 0.30, 0.10]
    time_from_start: {sec: 6, nanosec: 0}
"
```

### 4. Simulation Data Recording and Playback

#### 4.1 Script Description

- `rokae_hardware/scripts/record_sim.sh`  
  - Usage：`bash .../record_sim.sh [bagOutput Directory] [robot_typeOptional]`  
  - Default on `~/rosbags/<robotlowercase>_sim_Date and Time>` Create; the script will wait `/arm_controller/joint_trajectory` or `/position_joint_trajectory_controller/joint_trajectory` Start after it appears `ros2 bag record`，Record `/joint_states`、Two types of controllers `.../joint_trajectory` and `follow_joint_trajectory` action All topics、`/tf`、`/tf_static`、`/clock` etc. (see the full list in the script)）。  
- `rokae_hardware/scripts/replay_sim.sh`  
  - Usage：`bash .../replay_sim.sh <bagTable of Contents> [Playback speedrate] [with_clock true|false]`  
  - Default `rate=1.0`；The third parameter is `true` increase over time `ros2 bag play --clock`；Only replay the subset of topics related to trajectories。

#### 4.2 Recommendation process (three terminals)

The following is based on `robot_type:=CR7`、bag Table of Contents `~/rosbags/cr7_sim0002` As an example; please replace it with the local path and model. 

Terminal 1 — Start the simulation (without slider, consistent with recorded script habits)）

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch rokae_hardware gazebo_moveit.launch.py mode:=sim robot_type:=CR7 world:=empty.world enable_gui:=false enable_movej:=false
```

Keep running.

Terminal 2 — Start recording

```bash
source ~/ros2_ws/install/setup.bash

bash ~/ros2_ws/src/rokae_ros2/rokae_hardware/scripts/record_sim.sh ~/rosbags/cr7_sim0002 CR7
```

Stop recording: Press Ctrl+C on this terminal.

Terminal 3 — Send trajectory during recording (six axes; unified joint names）

```bash
source ~/ros2_ws/install/setup.bash

ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
points:
  - positions: [0.10, -0.20, 0.15, 0.00, 0.20, 0.00]
    time_from_start: {sec: 3, nanosec: 0}
  - positions: [0.30, -0.35, 0.25, 0.10, 0.30, 0.10]
    time_from_start: {sec: 6, nanosec: 0}
"
```

Can be performed multiple times during recording `ros2 topic pub`。Seven-axis whiskers `joint7` and `positions` for 7 numbers。

#### 4.3 Check bag

```bash
source ~/ros2_ws/install/setup.bash

ls ~/rosbags/

ros2 bag info ~/rosbags/cr7_sim0002
```

#### 4.4 Replay

1. Terminal 1 starts the same simulation as during recording again (the same `robot_type`）。
2. Terminal 2 — Play at Original Speed

```bash
source ~/ros2_ws/install/setup.bash

bash ~/ros2_ws/src/rokae_ros2/rokae_hardware/scripts/replay_sim.sh ~/rosbags/cr7_sim0002
```

3. Double speed

```bash
bash ~/ros2_ws/src/rokae_ros2/rokae_hardware/scripts/replay_sim.sh ~/rosbags/cr7_sim0002 2.0
```

4. bring `--clock`（third parameter `true`）

```bash
bash ~/ros2_ws/src/rokae_ros2/rokae_hardware/scripts/replay_sim.sh ~/rosbags/cr7_sim0002 1.0 true
```

#### 4.5 Usage Notes

1. Before Playback `robot_type`、URDF、The number of joint axes must be consistent with the recording.  
2. Use `ros2 control list_controllers` Confirm that the trajectory controller is active.  
3. Check for anomalies during playback `use_sim_time`、bag Does it contain `/clock`、and `replay_sim.sh` The third parameter.  
4. It is recommended to keep it `ros2 bag info` Output for record。
