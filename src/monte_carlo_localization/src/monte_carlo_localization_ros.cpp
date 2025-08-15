#include <monte_carlo_localization/monte_carlo_localization_ros.hpp>

MonteCarloLocalizationROS::MonteCarloLocalizationROS() : Node("monte_carlo_localization_node")
{
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period), std::bind(&MonteCarloLocalizationROS::timer_callback, this)
    );
}

void MonteCarloLocalizationROS::timer_callback()
{
	std::cout << "Testing timer callback" << std::endl;
}