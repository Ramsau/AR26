#include "tug_controller/tug_controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"
#include <nav2_util/geometry_utils.hpp>

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
  RCLCPP_INFO(node_->get_logger(), "TugController activated");

  // get robot geometry
  geometry_msgs::msg::TransformStamped tf_front_left = tf_buffer_->lookupTransform(base_link_frame_, front_left_frame_, tf2::TimePointZero);
  geometry_msgs::msg::TransformStamped tf_front_right = tf_buffer_->lookupTransform(base_link_frame_, front_right_frame_, tf2::TimePointZero);
  front_center_pose_.pose.position.x = (tf_front_left.transform.translation.x + tf_front_right.transform.translation.x) / 2;
  front_center_pose_.pose.position.y = (tf_front_left.transform.translation.y + tf_front_right.transform.translation.y) / 2;
  front_center_pose_.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
  front_center_pose_.header.frame_id = base_link_frame_;
  front_center_pose_.header.stamp = node_->get_clock()->now();

  wheelbase_ = std::sqrt(
    std::pow(tf_front_left.transform.translation.x - tf_front_right.transform.translation.x, 2) +
    std::pow(tf_front_left.transform.translation.y - tf_front_right.transform.translation.y, 2)
  );
}

// -----------------------------------------------------------------------------
void TugController::setPlan(const Path& path)
{
  RCLCPP_INFO(node_->get_logger(), "TugController received plan");
  path_ = path;
}

// -----------------------------------------------------------------------------
TugController::TwistStamped TugController::computeVelocityCommands(
  const PoseStamped& pose,
  const Twist& velocity,
  nav2_core::GoalChecker* goal_checker
)
{
  PoseStamped current_pose;
  try {
    front_center_pose_.header.stamp = node_->get_clock()->now();
    auto transform = tf_buffer_->lookupTransform(path_.header.frame_id, front_center_pose_.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(front_center_pose_, current_pose, transform);
    // tf_buffer_->transform(front_center_pose_, current_pose, path_.header.frame_id);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not transform robot pose to path frame: " << ex.what());
    return TwistStamped();
  }
  // 1. Lateral control
  int closest_path_idx = getClosestPathIndex(path_, current_pose);
  double path_angle = getPathAngle(path_, closest_path_idx, path_averaging_dist);
  double robot_angle = normalizeAngle(tf2::getYaw(current_pose.pose.orientation));
  double heading_error = normalizeAngle(path_angle - robot_angle);
  double cross_track_error = getCrossTrackError(path_, closest_path_idx, current_pose, path_angle);
  RCLCPP_INFO_STREAM(node_->get_logger(), "closest path idx: " << closest_path_idx);
  RCLCPP_INFO_STREAM(node_->get_logger(), "path angle: " << path_angle);
  RCLCPP_INFO_STREAM(node_->get_logger(), "robot angle: " << robot_angle);
  RCLCPP_INFO_STREAM(node_->get_logger(), "heading error: " << heading_error);
  RCLCPP_INFO_STREAM(node_->get_logger(), "cross track error: " << cross_track_error);

  // 2. Longitudinal control
  double distance_to_goal = nav2_util::geometry_utils::euclidean_distance(current_pose.pose.position, path_.poses[path_.poses.size() - 1].pose.position);
  double target_speed = distance_to_goal * k_speed_p_;
  target_speed = std::min(target_speed, speed_limit_);

  // 1b. steering angle
  double steering_angle = heading_error + std::atan2(cross_track_error * k_cte_, target_speed);
  RCLCPP_INFO_STREAM(node_->get_logger(), "steering angle: " << steering_angle);
  RCLCPP_INFO_STREAM(node_->get_logger(), "target speed: " << target_speed);

  RCLCPP_INFO_STREAM(node_->get_logger(), "");

  // 3. Compute command velocity
  double target_speed_x = target_speed * std::cos(steering_angle);
  double target_speed_y = target_speed * std::sin(steering_angle);
  double target_angular_rate = target_speed_y / wheelbase_;

  TwistStamped cmd_vel{};
  cmd_vel.twist.linear.x = target_speed_x;
  cmd_vel.twist.angular.z = target_angular_rate;

  return cmd_vel;
}

// -----------------------------------------------------------------------------
void TugController::setSpeedLimit(
  const double& speed_limit,
  const bool& percentage
)
{
  RCLCPP_INFO(node_->get_logger(), "TugController setting speed limit");
  speed_limit_ = speed_limit;
}

// -----------------------------------------------------------------------------
void TugController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "TugController deactivated");
}

// -----------------------------------------------------------------------------
void TugController::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "TugController cleanup");
}


int TugController::getClosestPathIndex(Path path, PoseStamped pose) {
  int closest_idx = -1;
  double closest_dist = std::numeric_limits<double>::max();



  for (int i = 0; i < path.poses.size(); i++) {
    PoseStamped path_pose = path.poses[i];
    double dist = std::sqrt(
        std::pow(path_pose.pose.position.x - pose.pose.position.x, 2) +
        std::pow(path_pose.pose.position.y - pose.pose.position.y, 2)
      );
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

double TugController::getPathAngle(Path path, int path_idx, double averaging_dist) {
  int path_idx_prev = std::max(path_idx - 1, 0);
  int path_idx_next = std::min<int>(path_idx + 1, path.poses.size() - 1);

  // average direction over distance
  while (path_idx_prev > 0 &&
    nav2_util::geometry_utils::euclidean_distance(
      path.poses[path_idx_prev - 1].pose.position, path.poses[path_idx].pose.position
      ) <= averaging_dist / 2) {
    path_idx_prev--;
  }
  while (path_idx_next < path.poses.size() - 1 &&
    nav2_util::geometry_utils::euclidean_distance(
      path.poses[path_idx].pose.position, path.poses[path_idx_next + 1].pose.position
      ) <= averaging_dist / 2) {
    path_idx_next++;
  }

  double angle = std::atan2(
    path.poses[path_idx_next].pose.position.y - path.poses[path_idx_prev].pose.position.y,
    path.poses[path_idx_next].pose.position.x - path.poses[path_idx_prev].pose.position.x
  );
  return angle;
}

double TugController::getCrossTrackError(Path path, int path_idx, PoseStamped pose, double path_angle) {
  PoseStamped path_pose = path.poses[path_idx];
  double path_dist = nav2_util::geometry_utils::euclidean_distance(path_pose.pose.position, pose.pose.position);
  double robot_to_path_angle = std::atan2(
    path_pose.pose.position.y - pose.pose.position.y,
    path_pose.pose.position.x - pose.pose.position.x
    );
  double angle_diff = normalizeAngle(path_angle - robot_to_path_angle);
  if (angle_diff > 0) {
    path_dist = -path_dist;
  }
  return path_dist;
}

double TugController::normalizeAngle(double angle) {
  angle = fmod(angle + M_PI, 2 * M_PI);
  if (angle < 0) {
    angle += 2 * M_PI;
  }
  return angle - M_PI;
}
} /* namespace tug_controller */

PLUGINLIB_EXPORT_CLASS(tug_controller::TugController, nav2_core::Controller);