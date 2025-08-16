#include <mc_localizer/mc_localizer_ros.hpp>

MCLocalizerRos::MCLocalizerRos() : Node("mc_localizer_node")
{
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period), std::bind(&MCLocalizerRos::timer_callback, this)
    );
}

void MCLocalizerRos::timer_callback()
{
	std::cout << "Testing timer callback" << std::endl;
}