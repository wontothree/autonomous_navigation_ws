#ifndef PARTICLE_H
#define PARTICLE_H

#include <mc_localizer/pose.hpp>

namespace mc_localizer {

class Particle {
private:
    Pose pose_;
    double weight_;

public:
    Particle(): pose_(0.0, 0.0, 0.0), weight_(0.0) {};

    Particle(double x, double y, double yaw, double weight): pose_(x, y, yaw), weight_(weight) {};

    Particle(Pose pose, double weight): pose_(pose), weight_(weight) {};

    ~Particle() {};

    inline double getX(void) { return pose_.getX(); }
    inline double getY(void) { return pose_.getY(); }
    inline double getYaw(void) { return pose_.getYaw(); }
    inline Pose getPose(void) { return pose_; }
    inline double getWeight(void) { return weight_; }

    inline void setX(double x) { pose_.setX(x); }
    inline void setY(double y) { pose_.setY(y); }
    inline void setYaw(double yaw) { pose_.setYaw(yaw); }
    inline void setPose(double x, double y, double yaw) { pose_.setPose(x, y, yaw); }
    inline void setPose(Pose p) { pose_.setPose(p); }
    inline void setWeight(double weight) { weight_ = weight; }
}; // class Particle

} // namespace mc_localizer

#endif // PARTICLE_H