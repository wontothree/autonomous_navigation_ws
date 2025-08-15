#include <monte_carlo_localization/monte_carlo_localization_ros.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::cout << "package monte_carlo_localization is started\n";

  rclcpp::spin(std::make_shared<MonteCarloLocalizationROS>());;

  rclcpp::shutdown();
  std::cout << "shutdown" << std::endl;

  return 0;
}