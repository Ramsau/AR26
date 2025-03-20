#include <memory>

#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"

#include "tug_laser_based_pf/LaserBasedPfNode.hpp"

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tug_laser_based_pf::LaserBasedPfNode>());
  rclcpp::shutdown();
  return 0;
}
