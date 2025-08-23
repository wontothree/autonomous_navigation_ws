#include <mc_localizer/mc_localizer.hpp>

namespace mc_localizer {

MCLocalizer::MCLocalizer()
{
    // pose
    // hyper parameter
    initial_pose_x_ = 0;
    initial_pose_y_ = 0;
    initial_pose_yaw_ = 0;
    initial_pose_noise_x_ = 0.02;
    initial_pose_noise_y_ = 0.02;
    initial_pose_noise_yaw_ = 0.02;

    // particles
    particle_num_ = 1000;

    // motion
    delta_x_ = 0.0;
    delta_y_ = 0.0;
    delta_distance_ = 0.0;
    delta_yaw_ = 0.0;
    is_omnidirectional_model = true;
    odom_noise_odm_ = {1.0, 0.5, 0.5, 0.5, 1.0, 0.5, 0.5, 0.5, 1.0};


    // step 1
    // degree to radian
    initial_pose_yaw_ *= M_PI / 180.0;

    // set initial pose
    mcl_estimated_pose_.set_pose(initial_pose_x_, initial_pose_y_, initial_pose_yaw_);
    initial_pose_noise_.set_pose(initial_pose_noise_x_, initial_pose_noise_y_, initial_pose_noise_yaw_);
    sample_particles(mcl_estimated_pose_, initial_pose_noise_);
    // resetReliabilities();
}

/**
 * @brief Gaussian random sampling using initial pose (step 1)
 * @member pose_tracking_particle_set_
 */
void MCLocalizer::sample_particles(
    const Pose &mean_pose,
    const Pose &standard_deviation_pose 
)
{
    // set up size of particle set
    pose_tracking_particle_set_.resize(particle_num_);

    // extract mean pose and standard deviation
    double mean_x = mean_pose.get_x();
    double mean_y = mean_pose.get_y();
    double mean_yaw = mean_pose.get_yaw();
    double standard_deviation_x = standard_deviation_pose.get_x();
    double standard_deviation_y = standard_deviation_pose.get_y();
    double standard_deviation_yaw = standard_deviation_pose.get_yaw();

    // set same weight (guarantee int)
    double weight = 1.0 / static_cast<double>(particle_num_);

    // generate particle set
    for (int i = 0; i < particle_num_; ++i) {
        double x = mean_x + nrand(standard_deviation_x);
        double y = mean_y + nrand(standard_deviation_y);
        double yaw = mean_yaw + nrand(standard_deviation_yaw);

        // store particle samples
        pose_tracking_particle_set_[i].set_pose(x, y, yaw);
        pose_tracking_particle_set_[i].set_weight(weight);
    }
}

// /**
//  * @member 
//  * @modifies
//  */
// void MCLocalizer::update_particles_by_motion_model(void)
// {
//     // delare and initialize local variables
//     double delta_x = delta_x_;
//     double delta_y = delta_y_;
//     double delta_distance = delta_x_;
//     double delta_yaw = delta_x_;

//     // initalize member variables
//     delta_x_ = 0;
//     delta_y_ = 0;
//     delta_distance_ = 0;
//     delta_yaw_ = 0;

//     // update member variables
//     delta_x_sum_ += fabs(delta_x);
//     delta_y_sum_ += fabs(delta_y);
//     delta_distance_sum_ += fabs(delta_distance);
//     delta_yaw_sum_ += fabs(delta_yaw);

//     if (!is_omnidirectional_model)
//     {
//         //
//     }
//     else
//     {  // omnidirectional model
//         double yaw = mcl_estimated_pose_.get_yaw();
//         double t = yaw + delta_yaw / 2.0;
//         double x = mcl_estimated_pose_.get_x() + delta_x * cos(t) + delta_y * cos(t + M_PI / 2.0f);
//         double y = mcl_estimated_pose_.get_y() + delta_x * sin(t) + delta_y * sin(t + M_PI / 2.0f);;
//         yaw += delta_yaw;
//         mcl_estimated_pose_.set_pose(x, y, yaw);

//         double square_x = delta_x * delta_x;
//         double square_y = delta_y * delta_y;
//         double square_yaw = delta_yaw * delta_yaw;
//         double xRandVal = square_x * odom_noise_odm_[0] + square_y * odom_noise_odm_[1] + square_yaw * odom_noise_odm_[2];
//         double yRandVal = square_x * odom_noise_odm_[3] + square_y * odom_noise_odm_[4] + square_yaw * odom_noise_odm_[5];
//         double yawRandVal = square_x * odom_noise_odm_[6] + square_y * odom_noise_odm_[7] + square_yaw * odom_noise_odm_[8];

//         for (int i = 0; i < particle_num_; ++i) {
//             double dx = delta_x + nrand(xRandVal);
//             double dy = delta_y + nrand(yRandVal);
//             double dyaw = delta_yaw + nrand(yawRandVal);

//             double yaw = pose_tracking_particle_set_[i].get_yaw();
//             double t = yaw + dyaw / 2.0;
//             double x = pose_tracking_particle_set_[i].get_x() + dx * cos(t) + dy * cos(t + M_PI / 2.0f);
//             double y = pose_tracking_particle_set_[i].get_y() + dx * sin(t) + dy * sin(t + M_PI / 2.0f);;
//             yaw += dyaw;
//             pose_tracking_particle_set_[i].set_pose(x, y, yaw);

//             // reliability transition model

//             // if (estimateReliability_) {
//             //     double decayRate = 1.0 - (relTransODM_[0] * dx * dx + relTransODM_[1] * dy * dy + relTransODM_[2] * dyaw * dyaw);
//             //     if (decayRate <= 0.0)
//             //         decayRate = 10.0e-6;
//             //     reliabilities_[i] *= decayRate;
//             // }
//         }
//     }
// }

// void MCLocalizer::calculate_likelihoods_by_measurement_model(void)
// {
//     // if (scanMightInvalid_)
//     //     return;

//     // if (rejectUnknownScan_ && (measurementModelType_ == 0 || measurementModelType_ == 1))
//     //     rejectUnknownScan();    

//     // sensor pose 계산
//     double x0 = base_link_to_laser_.get_x();
//     double y0 = base_link_to_laser_.get_y();
//     double yaw0 = base_link_to_laser_.get_yaw();
//     std::vector<Pose> sensor_poses(particle_num_);
//     for (int i = 0; i < particle_num_; ++i) {
//         double yaw = particles_[i].get_yaw();
//         double sensorX = x0 * cos(yaw) - y0 * sin(yaw) + particles_[i].get_x();
//         double sensorY = x0 * sin(yaw) - y0 * cos(yaw) + particles_[i].get_y();
//         double sensorYaw = yaw0 + yaw;
//         Pose sensorPose(sensorX, sensorY, sensorYaw);
//         sensor_poses[i] = sensorPose;
//         particles_[i].set_weight(0.0);
//     }

//     // measurement linkelihood
//     likelihoodShiftedSteps_.clear();
//     for (int i = 0; i < (int)scan_.ranges.size(); i += scanStep_) {
//         double range = scan_.ranges[i];
//         double rangeAngle = (double)i * scan_.angle_increment + scan_.angle_min;
//         double max;
//         for (int j = 0; j < particle_num_; ++j) {
//             double p;
//             if (measurementModelType_ == 0)
//                 p = calculateLikelihoodFieldModel(sensor_poses[j], range, rangeAngle);
//             else if (measurementModelType_ == 1)
//                 p = calculateBeamModel(sensor_poses[j], range, rangeAngle);
//             else
//                 p = calculateClassConditionalMeasurementModel(sensor_poses[j], rage, rangeAngle);

//             double w = particles_[j].getW();
//             w += log(p);
//             particles_[j].setW(w);
//             if (j == 0)
//                 max = w;
//             else {
//                 if (max < w)
//                     max = w;
//             }
//         }

//         // numerical stability
//         if (max < -300.0) {
//             for (int j = 0; j < particlesNum_; ++j) {
//                 double w = particles_[j].getW() + 300.0;
//                 particles_[j].set_weight(w);
//             }
//             likelihoodShiftedSteps_.push_back(true);
//         } else {
//             likelihoodShiftedSteps_.push_back(false);
//         }
//     }

//     // Weight normalization
//     double sum = 0.0;
//     double max;
//     int maxIdx;
//     for (int i = 0; i < particle_num_; ++i) {
//         // The log sum is converted to the probability.
//         double w = exp(particles_[i].get_weight());
//         particles_[i].set_weight(w);
//         sum += w;
//         if (i == 0) {
//             max = w;
//             maxIdx = i;
//         } else if (max < w) {
//             max = w;
//             maxIdx = i;
//         }
//     }
//     totalLikelihood_ = sum;
//     averageLikelihood_ = sum / (double)particlesNum_;
//     maxLikelihood_ = max;
//     maxLikelihoodParticleIdx_ = maxIdx;
// }

// void MCLocalizer::calculate_likelihoods_by_decision_model(void)
// {
//     double likelihood_sum = 0.0;
//     double max_likelihood;
//     int max_likelihood_particle_index;
//     for (int i = 0; i < particle_num_; ++i) {
//         Pose particle_pose = particles_[i].get_pose();
//         double measurement_likelihood = particle_[i].getW();
//         std::vector<double> residual_errors = getResidualErrors<double>(particle_pose);
//         double decision_likelihood;
//         if (1) { // check
//             double mae = mae_classifier_.getMAE(residual_errors);
//             decision_likelihood = mae_classifier_.calculate_decision_model(mae, &reliabilities_[i]);
//             mae_[i] = mae;
//         }

//         double w = measurement_likelihood * decision_likelihood;
//         particles_[i].set_weight(w);
//         likelihood_sum += w;
//         if (i == 0) {
//             max_likelihood = w;
//             max_likelihood_particle_index = i;
//         } else if (max_likelihood < w) {
//             max_likelihood = w;
//             max_likelihood_particle_index = i
//         }
//     }

//     total_likelihood_ = likelihood_sum;
//     average_likelihood_ = likelihood_sum / (double) particle_num_;
//     max_likelihood_ = max_likelihood;
//     max_likelihood_particle_index_ = max_likelihood_particle_index;
//     reliability_ = reliabilities_[max_likelihood_particle_index];
// }

// void MCLocalizer::calculate_likelihoods_from_global_localization(void)
// {
//     //
// }

// /**
//  * @member pose_tracking_particle_set_, global_localization_particle_set_
//  * @member pose_tracking_particle_num_, global_localization_particle_num_
//  * @member is_global_localization_sampling_enabled, can_use_global_localization_sample
//  * 
//  * @modifies mcl_estimated_pose_
//  */
// void MCLocalizer::estimate_robot_pose(void)
// {
//     // if (scanMightInvalid_)
//     //     return;

//     double temp_yaw = mcl_estimated_pose_.get_yaw();
//     double x = 0.0, y = 0.0, yaw = 0.0;
//     double sum = 0.0;
//     for (int i = 0; i < pose_tracking_particle_num_; ++i) {
//         double weight = pose_tracking_particle_set_[i].get_weight();
//         x += pose_tracking_particle_set_[i].get_x() * weight;
//         y += pose_tracking_particle_set_[i].get_y() * weight;
        
//         double dyaw = temp_yaw - pose_tracking_particle_set_[i].get_yaw();

//         while (dyaw < - M_PI)
//             dyaw += 2.0 * M_PI;
//         while (dyaw > M_PI)
//             dyaw -= 2.0 * M_PI;

//         yaw += dyaw * weight;
//         sum += weight;
//     }

//     if (is_global_localization_sampling_enabled && can_use_global_localization_sample) {
//         double x2 = x, y2 = y, yaw2 = yaw;
//         for (int i = 0; i < global_localization_particle_num_; ++i) {
//             double weight = global_localization_particle_set_[i].getWeight();
//             x += global_localization_particle_set_[i].get_x() * weight;
//             y += global_localization_particle_set_[i].get_y() * weight;

//             double dyaw = temp_yaw - global_localization_particle_set_[i].getYaw();
//             while (dyaw < -M_PI)
//                 dyaw += 2.0 * M_PI;
//             while (dyaw > M_PI)
//                 dyaw -= 2.0 * M_PI;

//             yaw += dyaw * weight;
//             sum += weight;
//         }
//         if (sum > 1.0)
//             x = x2, y = y2, yaw = yaw2;
//     }

//     yaw = temp_yaw - yaw;
//     mcl_estimated_pose_.set_pose(x, y, yaw);
// }

void MCLocalizer::resample_particles(void)
{
    //
}

} // namespace mc_localizer
