# Architecture

---

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
||initialize_particle_set||||
||update_particles_by_motion_model||omnidirectional model을 사용하여 `pose_tracking_particle_set_` 업데이트|||
||calculate_likelihoods_by_measurement_model||||
||calculate_likelihoods_by_decision_model||||
||calculate_likelihoods_from_global_localization||||
||estimate_robot_pose||||
||resample_particles||||

---

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
