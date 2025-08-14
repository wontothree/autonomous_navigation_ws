# autonomous_navigation_ws

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
