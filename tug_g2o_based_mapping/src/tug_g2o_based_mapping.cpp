#include <memory>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"

#include "tug_g2o_based_mapping/G2OBasedMapping.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<tug_g2o_based_mapping::G2OBasedMapping> node 
    = std::make_shared<tug_g2o_based_mapping::G2OBasedMapping>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
