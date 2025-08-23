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
    │   ├── mc_localizer_ros.cpp                       # 
    │   └── mc_localizer.cpp                           # 
    ├── CMakeLists.txt                             
    └── package.xml    

# Subscribed Topics

Following messages (topics) are needed to be published;

- sensor_msgs/msg/LaserScan (`/scan`)
- nav_msgs/msg/Odometry (`/odom`)
- nav_msgs/msg/OccupancyGrid (`/map`)

Also, static transformation between following two frames is needed to be set.

- origin of a robot (base_link)
- 2D LiDAR (laser)

# Published Topics

# `mc_localizer.h` and `mc_localizer.cpp`

## Member Variables

|Index|Type|Variable|Description|
|---|---|---|---|
||`std::vector<Particle>`|`pose_tracking_particle_set_`||
||`std::vector<Particle>`|`global_localization_particle_set_`||
||int|`pose_tracking_particle_num_`||
||int|`global_localization_particle_num_`||
|||||

## Functions

Logic

|Return|Function|Input|Description|사용하는 Member Variables|
|---|---|---|---|---|
||sample_particles||||
||update_particles_by_motion_model||omnidirectional model을 사용하여 `pose_tracking_particle_set_` 업데이트|||
||calculate_likelihoods_by_measurement_model||||
||calculate_likelihoods_by_decision_model||||
||calculate_likelihoods_from_global_localization||||
||estimate_robot_pose||||
||resample_particles||||

# `mc_localizer_ros.hpp`

## Member Variables

|Index|Type|Variable|Initialization|Description|
|---|---|---|---|---|
||std::string|scan_topic_name_|||
||std::string|odom_topic_name_|||

## Functions

|Return|Function|Input|Description|사용하는 Member Variables|
|---|---|---|---|---|
||callback_timer||||
||callback_scan||||
||callback_odom||||
||callback_initial_pose||||
