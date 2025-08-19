#ifndef MC_LOCALIZER_H
#define MC_LOCALIZER_H

// #include <cmath> // fabs, rand, cos, log, M_PI
#include <vector> // std::vector

#include "sensor_msgs/msg/laser_scan.hpp"

#include <mc_localizer/pose.hpp>
#include <mc_localizer/particle.hpp>
#include <mc_localizer/mae_classifier.hpp>

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

    // localizatoin result
    double total_likelihood_, average_likelihood_, max_likelihood_;
    int max_likelihood_particle_index_;

    // reliability estimation
    double reliability_;
    std::vector<double> reliabilities_;

    // mean absolute error (MAE)-based failure detector
    MAEClassifier mae_classifier_;

public:
    void update_particle_by_motion_model(void);

    void calculate_likelihoods_by_measurement_model(void);

    void calculate_likelihoods_by_decision_model(void);

private:
    // util functions
    inline double nrand(double n)
    {
        return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
    }

    template <typename T>
    std::vector<T> getResidualErrors(Pose pose) {
        double yaw = pose.getYaw();
        double sensorX = baseLink2Laser_.getX() * cos(yaw) - baseLink2Laser_.getY() * sin(yaw) + pose.getX();
        double sensorY = baseLink2Laser_.getX() * sin(yaw) + baseLink2Laser_.getY() * cos(yaw) + pose.getY();
        double sensorYaw = baseLink2Laser_.getYaw() + yaw;
        int size = (int)scan_.ranges.size();
        std::vector<T> residualErrors(size);
        for (int i = 0; i < size; ++i) {
            double r = scan_.ranges[i];
            if (r <= scan_.range_min || scan_.range_max <= r) {
                residualErrors[i] = -1.0;
                continue;
            }
            double t = (double)i * scan_.angle_increment + scan_.angle_min + sensorYaw;
            double x = r * cos(t) + sensorX;
            double y = r * sin(t) + sensorY;
            int u, v;
            xy2uv(x, y, &u, &v);
            if (onMap(u, v)) {
                T dist = (T)distMap_.at<float>(v, u);
                residualErrors[i] = dist;
            } else {
                residualErrors[i] = -1.0;
            }
        }
        return residualErrors;
    }

}; // class mc_localizer

} // namespace mc_localizer

#endif // MC_LOCALIZER_H