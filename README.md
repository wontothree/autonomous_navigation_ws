# autonomous_navigation_ws

Autonomous Navigation of Mobile Robot with 2D LiDAR and IMU.

- [Localization] Monte Carlo Localization
- [Local Planner] Model Predictive Path Integral Control

# Tested Environment

- Mac M1
- ROS2 Humble

# Dependencies

rviz

```bash
apt-get update
apt-get install -y tigervnc-standalone-server tigervnc-common novnc websockify
websockify --web=/usr/share/novnc/ 8080 localhost:5901

source /opt/ros/humble/setup.bash
. install/local_setup.bash
rviz2
```

http://localhost:8080/vnc.html

# Getting Started

```bash
source /opt/ros/humble/setup.bash
. install/local_setup.bash
```
