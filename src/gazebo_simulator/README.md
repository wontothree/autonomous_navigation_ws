# [Package] gazebo_simulator

    gazebo_simulator
    ├── GazeboFiles                     # gazebo files
    │   ├── models
    │   └── world
    ├── meshes
    │   └── hokuyo.dae                  # 3d model blueprint
    ├── urdf                            # robot model
    │   ├── diffbot.gazebo              # gazebo plugin
    │   ├── diffbot.urdf                # useless
    │   └── diffbot.urdf.xacro          # main
    ├── worlds
    │   └── simulator.world             # whole simulation environment (entry point)
    │    
    ├── launch
    ├── include
    ├── src
    ├── CMakeLists.txt
    └── package.xml

    ├── rviz
    │   ├── description.rviz
    │   ├── diffbot.rviz
    │   ├── skibot.rviz
    │   └── tinybot.rviz

# Quick Start

```bash
apt install ros-humble-joint-state-publisher
ros2 pkg list | grep joint_state_publisher
```

# Teleoperation

```bash
# dependencies
apt-get update
apt-get install -y ros-humble-teleop-twist-keyboard

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diffbot/cmd_vel
```

# Reference

https://github.com/Road-Balance/gcamp_ros2_basic
