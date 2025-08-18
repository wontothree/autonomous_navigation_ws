#ifndef MC_LOCALIZER_H
#define MC_LOCALIZER_H

// #include <cmath> // fabs, rand, cos, log, M_PI
#include <vector> // std::vector

#include "sensor_msgs/msg/laser_scan.hpp"

#include <mc_localizer/pose.hpp>
#include <mc_localizer/particle.hpp>

namespace mc_localizer {

class MCLocalizer {
public:
    MCLocalizer();
    ~MCLocalizer() {};

private:
    // pose
    double inital_pose_x_, inital_pose_y_, inital_pose_yaw_;
    Pose mc_localizer_pose_;
    Pose base_link_to_laser_;

    // particles
    int particle_num_;
    std::vector<Particle> particles_;

    // motion
    double delta_x_, delta_y_, delta_distance_, delta_yaw_;
    double delta_x_sum_, delta_y_sum_, delta_distance_sum_, delta_yaw_sum_, delta_time_sume_;
    bool is_omnidirectional_model;
    std::vector<double> odom_noise_odm_;

    // measurements
    std::vector<bool> likelihoodShiftedSteps_;
    sensor_msgs::msg::LaserScan scan_;

    // measurement model
    int scanStep_;

public:
    void update_particle_by_motion_model(void);

    void calculate_likelihoods_measurement_model(void);

private:
    // util functions
    inline double nrand(double n)
    {
        return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
    }

}; // class mc_localizer

} // namespace mc_localizer

#endif // MC_LOCALIZER_H