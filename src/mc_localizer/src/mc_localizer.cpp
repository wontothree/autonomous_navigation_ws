#include <mc_localizer/mc_localizer.hpp>

namespace mc_localizer {

MCLocalizer::MCLocalizer()
{
    // pose
    inital_pose_x_ = 0;
    inital_pose_y_ = 0;
    inital_pose_yaw_ = 0;
    mc_localizer_pose_.setPose(inital_pose_x_, inital_pose_y_, inital_pose_yaw_);

    // particles
    particle_num_ = 100;

    // motion
    delta_x_ = 0.0;
    delta_y_ = 0.0;
    delta_distance_ = 0.0;
    delta_yaw_ = 0.0;
    is_omnidirectional_model = true;
    odom_noise_odm_ = {1.0, 0.5, 0.5, 0.5, 1.0, 0.5, 0.5, 0.5, 1.0};
}

/**
 * @member 
 * @modifies
 */
void MCLocalizer::update_particle_by_motion_model(void)
{
    // delare and initialize local variables
    double delta_x = delta_x_;
    double delta_y = delta_y_;
    double delta_distance = delta_x_;
    double delta_yaw = delta_x_;

    // initalize member variables
    delta_x_ = 0;
    delta_y_ = 0;
    delta_distance_ = 0;
    delta_yaw_ = 0;

    // update member variables
    delta_x_sum_ += fabs(delta_x);
    delta_y_sum_ += fabs(delta_y);
    delta_distance_sum_ += fabs(delta_distance);
    delta_yaw_sum_ += fabs(delta_yaw);

    if (!is_omnidirectional_model)
    {
        //
    }
    else
    {  // omnidirectional model
        double yaw = mc_localizer_pose_.getYaw();
        double t = yaw + delta_yaw / 2.0;
        double x = mc_localizer_pose_.getX() + delta_x * cos(t) + delta_y * cos(t + M_PI / 2.0f);
        double y = mc_localizer_pose_.getY() + delta_x * sin(t) + delta_y * sin(t + M_PI / 2.0f);;
        yaw += delta_yaw;
        mc_localizer_pose_.setPose(x, y, yaw);

        double square_x = delta_x * delta_x;
        double square_y = delta_y * delta_y;
        double square_yaw = delta_yaw * delta_yaw;
        double xRandVal = square_x * odom_noise_odm_[0] + square_y * odom_noise_odm_[1] + square_yaw * odom_noise_odm_[2];
        double yRandVal = square_x * odom_noise_odm_[3] + square_y * odom_noise_odm_[4] + square_yaw * odom_noise_odm_[5];
        double yawRandVal = square_x * odom_noise_odm_[6] + square_y * odom_noise_odm_[7] + square_yaw * odom_noise_odm_[8];

        for (int i = 0; i < particle_num_; ++i) {
            double dx = delta_x + nrand(xRandVal);
            double dy = delta_y + nrand(yRandVal);
            double dyaw = delta_yaw + nrand(yawRandVal);

            double yaw = particles_[i].getYaw();
            double t = yaw + dyaw / 2.0;
            double x = particles_[i].getX() + dx * cos(t) + dy * cos(t + M_PI / 2.0f);
            double y = particles_[i].getY() + dx * sin(t) + dy * sin(t + M_PI / 2.0f);;
            yaw += dyaw;
            particles_[i].setPose(x, y, yaw);

            // reliability transition model

            // if (estimateReliability_) {
            //     double decayRate = 1.0 - (relTransODM_[0] * dx * dx + relTransODM_[1] * dy * dy + relTransODM_[2] * dyaw * dyaw);
            //     if (decayRate <= 0.0)
            //         decayRate = 10.0e-6;
            //     reliabilities_[i] *= decayRate;
            // }
        }
    }
}

} // namespace mc_localizer