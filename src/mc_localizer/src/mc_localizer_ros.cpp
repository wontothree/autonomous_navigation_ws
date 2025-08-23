#include "mc_localizer/mc_localizer_ros.hpp"

MCLocalizerROS::MCLocalizerROS() : Node("mc_localizer_node")
{
    // subscriber
    scan_topic_name_ = "/diffbot/scan";
    odom_topic_name_ = "/diffbot/odom";
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_name_,
        10,
        std::bind(&MCLocalizerROS::callback_scan, this, std::placeholders::_1)
    );
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name_,
        10,
        std::bind(&MCLocalizerROS::callback_odom, this, std::placeholders::_1)
    );

    // timer callback (main execute)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period),
        std::bind(&MCLocalizerROS::callback_timer, this)
    );
}

void MCLocalizerROS::callback_timer()
{
    //
}

void MCLocalizerROS::callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
{
    if (1) // canUpdateScan_
        scan_ = *scan;
    // if (!gotScan_)
    //     gotScan_ = true;
}

void MCLocalizerROS::callback_odom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    std::cout << "odom callback" << std::endl;
}