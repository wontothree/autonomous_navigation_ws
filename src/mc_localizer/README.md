# [Package] mc_localizer

Monte Carlo Localization Module

    als_ros
    ├── include/als_ros     
        ├── mc_localizer_ros.h    
        ├── mc_localizer.h                             # logic
        ├── mae_classifier.h                           # 
        ├── histogram.h
        ├── particle.h                                 # [data struecture] 한 입자 (x, y, yaw, weight)
        ├── pose.h                                     # [data struecture] 한 자세 (x, y, yaw)
        └── point.h                                    # [data struecture] 한 점 (x, y)
    ├── launch/                            
    ├── src/     
        ├── mc_localizer_node.cpp                      # 
        ├── mc_localizer_ros.cpp                       # 
        └── mc_localizer.cpp                           # 
    ├── CMakeLists.txt                             
    └── package.xml    

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

|Index|Function|Description|사용하는 Member Variables|
|---|---|---|---|
||`update_particles_by_motion_model`|omnidirectional model을 사용하여 `pose_tracking_particle_set_` 업데이트||
||`calculate_likelihoods_by_measurement_model`|||
||`calculate_likelihoods_by_decision_model`|||
||`calculate_likelihoods_from_global_localization`|||
||`estimate_robot_pose`|||
||`resample_particles`|||
