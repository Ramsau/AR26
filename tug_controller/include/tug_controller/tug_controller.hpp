#ifndef _TUG_CONTROLLER__TUG_CONTROLLER_HPP_
#define _TUG_CONTROLLER__TUG_CONTROLLER_HPP_

#include "nav2_core/controller.hpp"
#include <nav2_costmap_2d/footprint_collision_checker.hpp>

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
    Path path_;
    PoseStamped front_center_pose_;
    PoseStamped front_left_pose_ ;
    PoseStamped front_right_pose_;
    PoseStamped base_link_pose_;
    double wheelbase_ = 0.0;
    std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>> costmap_cheker_;

  // Settings -----------------------------------------------------------------
  private:
    std::string front_left_frame_ = "front_left_wheel";
    std::string front_right_frame_ = "front_right_wheel";
    std::string base_link_frame_ = "base_link";
    double path_averaging_dist = 0.1;
    double k_cte_ = 0.5;
    double speed_limit_ = 1.0;
    double max_acceleration_ = 0.3;
    double max_steering_angle_ = M_PI_2;
    double movement_cutoff_distance_ = 0.1;
    double k_speed_distance_ = 1.0;
    double k_speed_p_ = 1.0;
    double k_speed_i_ = 0.01;
    double collision_lookahead_time_ = 0.5;
    double collision_costmap_value = 200;
    double collision_speed_step_factor = 0.5;
    double collision_angular_rate_step_factor = 0.9;

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

  // Custom Methods -------------------------------------------------------------------
  private:
    int getClosestPathIndex(Path path, PoseStamped pose);
    double getPathAngle(Path path, int path_idx, double averaging_dist);
    double getCrossTrackError(Path path, int path_idx, PoseStamped pose, double path_angle);
    double normalizeAngle(double angle);
    double getPoseCost(PoseStamped pose);
};

} /* namespace tug_controller */

#endif /* _TUG_CONTROLLER__TUG_CONTROLLER_HPP_ */