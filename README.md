<div align="center">

  # Autonomous Navigation ROS2 Humble-Based Workspace
  
  Autonomous Navigation of Indoor Mobile Robot with 2D LiDAR and IMU
  
  [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
  [![Node.js](https://img.shields.io/badge/Node.js-22.16.0-green.svg)](https://nodejs.org/)
  [![Status](https://img.shields.io/badge/Status-Alpha-orange.svg)]()
  
</div>

--- 

## 🚀 Quick Start

```bash
source /opt/ros/humble/setup.bash
. install/local_setup.bash
```

```bash
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

---

## System Requirement

- ROS2 Humble

## Tested Environment

- Mac M1

## 🏗️ Technical Architecture

- [Localization] Monte Carlo Localization
- [Local Planner] Model Predictive Path Integral Control

---

## 📁 Project Structure

    autonomous_navigation_ws
    └── src/
        ├── monte_carlo_localization/             # localization
        └── mppi_planner/                         # local planner

---