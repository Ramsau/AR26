#ifndef _TUG_CONTROLLER__TUG_CONTROLLER_HPP_
#define _TUG_CONTROLLER__TUG_CONTROLLER_HPP_

#include "nav2_core/controller.hpp"

namespace tug_controller
{

class TugController : public nav2_core::Controller
{
  // Directives ----------------------------------------------------------------
  private:
    // Geometry messages
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using Twist = geometry_msgs::msg::Twist;
    using TwistStamped = geometry_msgs::msg::TwistStamped;

    // Nav messages
    using Path = nav_msgs::msg::Path;

  // Variables -----------------------------------------------------------------
  private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string controller_name_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> cost_map_;

  // Methods -------------------------------------------------------------------
  public:
    TugController() = default;
    ~TugController() = default;

    virtual void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& node,
      std::string controller_name,
      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> cost_map
    ) override;

    virtual void activate() override;
    virtual void setPlan(const Path& path) override;

    virtual TwistStamped computeVelocityCommands(
      const PoseStamped& pose,
      const Twist& velocity,
      nav2_core::GoalChecker* goal_checker
    ) override;

    virtual void setSpeedLimit(
      const double& speed_limit,
      const bool& percentage
    ) override;

    virtual void deactivate() override;
    virtual void cleanup() override;
};

} /* namespace tug_controller */

#endif /* _TUG_CONTROLLER__TUG_CONTROLLER_HPP_ */