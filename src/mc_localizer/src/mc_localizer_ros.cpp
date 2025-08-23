#include "mc_localizer/mc_localizer_ros.hpp"

MCLocalizerRos::MCLocalizerRos() : Node("mc_localizer_node")
{
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period), std::bind(&MCLocalizerRos::callback_timer, this)
    );

    // subscriber
    scan_topic_name_ = "/scan";
    odom_topic_name_ = "/odom/";
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_name_,
        10,
        std::bind(&MCLocalizerRos::callback_scan, this, std::placeholders::_1)
    );
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name_,
        10,
        std::bind(&MCLocalizerRos::callback_odom, this, std::placeholders::_1)
    );
}

void MCLocalizerRos::callback_timer()
{
	std::cout << "Testing timer callback" << std::endl;
}

void MCLocalizerRos::callback_scan(sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    //
}

void MCLocalizerRos::callback_odom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    //
}