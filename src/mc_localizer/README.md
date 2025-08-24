# [Package] mc_localizer

Monte Carlo Localization Module

    mc_localizer
    ├── include/mc_localizer     
    │   ├── mc_localizer_ros.h                         # callback, visualization, and ROS functions
    │   ├── mc_localizer.h                             # logic
    │   ├── mae_classifier.h                           # 
    │   ├── histogram.h
    │   ├── particle.h                                 # [data struecture] 한 입자 (x, y, yaw, weight)
    │   ├── pose.h                                     # [data struecture] 한 자세 (x, y, yaw)
    │   └── point.h                                    # [data struecture] 한 점 (x, y)
    ├── launch/                            
    ├── src/     
    │   ├── mc_localizer_node.cpp                      # entry point
    │   ├── mc_localizer_ros.cpp                       # ros code (subscribers, publishers, callback and visualization functions)
    │   └── mc_localizer.cpp                           # 
    ├── CMakeLists.txt                             
    └── package.xml    

# Subscribed Topics

Following messages (topics) are needed to be published;

- [sensor_msgs/msg/LaserScan](https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html) (`/scan`) 
- [nav_msgs/msg/Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html) (`/odom`)
- [nav_msgs/msg/OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html) (`/map`)

Also, static transformation between following two frames is needed to be set.

- origin of a robot (base_link)
- 2D LiDAR (laser)

# Published Topics

# Nodes

`mc_localizer_node`
