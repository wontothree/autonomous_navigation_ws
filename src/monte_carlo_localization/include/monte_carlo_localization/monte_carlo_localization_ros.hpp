#ifndef MONTE_CARLO_LOCALIZATION_ROS_H
#define MONTE_CARLO_LOCALIZATION_ROS_H

#include "rclcpp/rclcpp.hpp"

class MonteCarloLocalizationROS : public rclcpp::Node {
public:
    MonteCarloLocalizationROS();
    ~MonteCarloLocalizationROS() {};

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

#endif // MONTE_CARLO_LOCALIZATION_ROS_H