#ifndef MC_LOCALIZER_ROS_H
#define MC_LOCALIZER_ROS_H

#include <rclcpp/rclcpp.hpp>              // rclcpp::Node
#include <sensor_msgs/msg/laser_scan.hpp> // sensor_msgs::msg::LaserScan
#include <nav_msgs/msg/odometry.hpp>      // nav_msgs::msg::Odometry

class MCLocalizerROS : public rclcpp::Node {
public:
    MCLocalizerROS();
    ~MCLocalizerROS() {};

private:
    const int timer_period = 10;             // timer period (ms)
    rclcpp::TimerBase::SharedPtr timer_;

    // subscriber
    std::string scan_topic_name_, odom_topic_name_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    // callback_scan
    sensor_msgs::msg::LaserScan scan_;

private:
    void callback_timer();

    /**
     * @members scan_
     */
    void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr &scan);

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr odom);
};

#endif // MC_LOCALIZER_ROS_H