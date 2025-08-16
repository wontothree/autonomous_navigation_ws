#ifndef MC_LOCALIZER_ROS_H
#define MC_LOCALIZER_ROS_H

#include "rclcpp/rclcpp.hpp"

class MCLocalizerRos : public rclcpp::Node {
public:
    MCLocalizerRos();
    ~MCLocalizerRos() {};

private:
    const int timer_period = 10;                                  // timer period (ms)

private:
    /**
     * @brief This function is called in constant period of timer
     */
    // timer
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
};

#endif // MC_LOCALIZER_ROS_H