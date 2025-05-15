#include "tug_controller/tug_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace tug_controller
{

// -----------------------------------------------------------------------------
void TugController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr& node,
  std::string controller_name,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> cost_map
)
{
  node_ = node.lock();
  controller_name_ = controller_name;
  tf_buffer_ = tf_buffer;
  cost_map_ = cost_map;
}

// -----------------------------------------------------------------------------
void TugController::activate()
{

}

// -----------------------------------------------------------------------------
void TugController::setPlan(const Path& path)
{

}

// -----------------------------------------------------------------------------
TugController::TwistStamped TugController::computeVelocityCommands(
  const PoseStamped& pose,
  const Twist& velocity,
  nav2_core::GoalChecker* goal_checker
)
{
  return TwistStamped();
}

// -----------------------------------------------------------------------------
void TugController::setSpeedLimit(
  const double& speed_limit,
  const bool& percentage
)
{
  
}

// -----------------------------------------------------------------------------
void TugController::deactivate()
{

}

// -----------------------------------------------------------------------------
void TugController::cleanup()
{

}

} /* namespace tug_controller */

PLUGINLIB_EXPORT_CLASS(tug_controller::TugController, nav2_core::Controller);