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

void MCLocalizer::calculate_likelihoods_by_measurement_model(void)
{
    // if (scanMightInvalid_)
    //     return;

    // if (rejectUnknownScan_ && (measurementModelType_ == 0 || measurementModelType_ == 1))
    //     rejectUnknownScan();    

    // sensor pose 계산
    double x0 = base_link_to_laser_.getX();
    double y0 = base_link_to_laser_.getY();
    double yaw0 = base_link_to_laser_.getYaw();
    std::vector<Pose> sensor_poses(particle_num_);
    for (int i = 0; i < particle_num_; ++i) {
        double yaw = particles_[i].getYaw();
        double sensorX = x0 * cos(yaw) - y0 * sin(yaw) + particles_[i].getX();
        double sensorY = x0 * sin(yaw) - y0 * cos(yaw) + particles_[i].getY();
        double sensorYaw = yaw0 + yaw;
        Pose sensorPose(sensorX, sensorY, sensorYaw);
        sensor_poses[i] = sensorPose;
        particles_[i].setW(0.0);
    }

    // measurement linkelihood
    likelihoodShiftedSteps_.clear();
    for (int i = 0; i < (int)scan_.ranges.size(); i += scanStep_) {
        double range = scan_.ranges[i];
        double rangeAngle = (double)i * scan_.angle_increment + scan_.angle_min;
        double max;
        for (int j = 0; j < particle_num_; ++j) {
            double p;
            if (measurementModelType_ == 0)
                p = calculateLikelihoodFieldModel(sensor_poses[j], range, rangeAngle);
            else if (measurementModelType_ == 1)
                p = calculateBeamModel(sensor_poses[j], range, rangeAngle);
            else
                p = calculateClassConditionalMeasurementModel(sensor_poses[j], rage, rangeAngle);

            double w = particles_[j].getW();
            w += log(p);
            particles_[j].setW(w);
            if (j == 0)
                max = w;
            else {
                if (max < w)
                    max = w;
            }
        }

        // numerical stability
        if (max < -300.0) {
            for (int j = 0; j < particlesNum_; ++j) {
                double w = particles_[j].getW() + 300.0;
                particles_[j].setW(w);
            }
            likelihoodShiftedSteps_.push_back(true);
        } else {
            likelihoodShiftedSteps_.push_back(false);
        }
    }

    // Weight normalization
    double sum = 0.0;
    double max;
    int maxIdx;
    for (int i = 0; i < particlesNum_; ++i) {
        // The log sum is converted to the probability.
        double w = exp(particles_[i].getW());
        particles_[i].setW(w);
        sum += w;
        if (i == 0) {
            max = w;
            maxIdx = i;
        } else if (max < w) {
            max = w;
            maxIdx = i;
        }
    }
    totalLikelihood_ = sum;
    averageLikelihood_ = sum / (double)particlesNum_;
    maxLikelihood_ = max;
    maxLikelihoodParticleIdx_ = maxIdx;
}

void calculate_likelihoods_by_decision_model(void)
{
    double likelihood_sum = 0.0;
    double max_likelihood;
    int max_likelihood_particle_index;
    for (int i = 0; i < particle_num_; ++i) {
        Pose particle_pose = particles_[i].getPose();
        double measurement_likelihood = particle_[i].getW();
        std::vector<double> residual_errors = getResidualErrors<double>(particle_pose);
        double decision_likelihood;
        if (1) { // check
            double mae = mae_classifier_.getMAE(residual_errors);
            decision_likelihood = mae_classifier_.calculate_decision_model(mae, &reliabilities_[i]);
            mae_[i] = mae;
        }

        double w = measurement_likelihood * decision_likelihood;
        particles_[i].setW(w);
        likelihood_sum += w;
        if (i == 0) {
            max_likelihood = w;
            max_likelihood_particle_index = i;
        } else if (max_likelihood < w) {
            max_likelihood = w;
            max_likelihood_particle_index = i
        }
    }

    total_likelihood_ = likelihood_sum;
    average_likelihood_ = likelihood_sum / (double) particles_num_;
    max_likelihood_ = max_likelihood;
    max_likelihood_particle_index_ = max_likelihood_particle_index;
    reliability_ = reliabilities_[max_likelihood_particle_index];
}

} // namespace mc_localizer
