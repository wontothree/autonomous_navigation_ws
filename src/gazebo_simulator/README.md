# [Package] gazebo_simulator

    gazebo_simulator
    ├── GazeboFiles                     # gazebo files
    │   ├── models
    │   └── world
    ├── meshes
    │   └── hokuyo.dae                  # 3d model blueprint
    ├── urdf                            # robot model
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

```bash
conda install -c robostack ros-humble-joint-state-publisher
ros2 pkg list | grep joint_state_publisher
```

```bash
colcon build
source install/setup.zsh
gazebo install/gazebo_simulator/share/gazebo_simulator/worlds/simulator.world

ros2 launch gazebo_simulator robot_world.launch.py
```

```bash
ps aux | grep gz
pkill -f gz

gazebo --verbose empty.world
```