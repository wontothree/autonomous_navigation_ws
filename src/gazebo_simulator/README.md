# [Package] gazebo_simulator

    gazebo_simulator
    ├── GazeboFiles                     # gazebo files
    │   ├── models
    │   └── world
    ├── meshes
    │   └── hokuyo.dae                  # 3d model blueprint
    ├── urdf                            # robot model
    │   ├── diffbot.gazebo              # gazebo plugin
    │   ├── diffbot.urdf
    │   └── diffbot.urdf.xacro          # main
    ├── worlds
    │   └── simulator.world             # whole simulation environment (entry point)
    │    

    ├── CMakeLists.txt
    └── package.xml

    ├── rviz
    │   ├── description.rviz
    │   ├── diffbot.rviz
    │   ├── skibot.rviz
    │   └── tinybot.rviz

    ├── launch
    ├── include
    ├── src

# Quick Start


# Docker

```bash
apt install ros-humble-joint-state-publisher
ros2 pkg list | grep joint_state_publisher
```
