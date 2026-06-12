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
  costmap_cheker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>(cost_map_->getCostmap());
}

// -----------------------------------------------------------------------------
void TugController::activate()
{

  // get robot geometry
  geometry_msgs::msg::TransformStamped tf_front_left = tf_buffer_->lookupTransform(base_link_frame_, front_left_frame_, tf2::TimePointZero);
  geometry_msgs::msg::TransformStamped tf_front_right = tf_buffer_->lookupTransform(base_link_frame_, front_right_frame_, tf2::TimePointZero);
  front_center_pose_.pose.position.x = (tf_front_left.transform.translation.x + tf_front_right.transform.translation.x) / 2;
  front_center_pose_.pose.position.y = (tf_front_left.transform.translation.y + tf_front_right.transform.translation.y) / 2;
  front_center_pose_.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
  front_center_pose_.header.frame_id = base_link_frame_;
  front_center_pose_.header.stamp = node_->get_clock()->now();

  front_left_pose_ = front_center_pose_;
  front_left_pose_.pose.position.x = tf_front_left.transform.translation.x;
  front_left_pose_.pose.position.y = tf_front_left.transform.translation.y;

  front_right_pose_ = front_center_pose_;
  front_right_pose_.pose.position.x = tf_front_right.transform.translation.x;
  front_right_pose_.pose.position.y = tf_front_right.transform.translation.y;

  base_link_pose_ = front_center_pose_;
  base_link_pose_.pose.position.x = 0;
  base_link_pose_.pose.position.y = 0;

  wheelbase_ = std::sqrt(
    std::pow(tf_front_left.transform.translation.x - tf_front_right.transform.translation.x, 2) +
    std::pow(tf_front_left.transform.translation.y - tf_front_right.transform.translation.y, 2)
  );
}

// -----------------------------------------------------------------------------
void TugController::setPlan(const Path& path)
{
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
  PoseStamped current_base_link;

  geometry_msgs::msg::TransformStamped base_to_map;
  try {
    base_to_map = tf_buffer_->lookupTransform(path_.header.frame_id, base_link_frame_, tf2::TimePointZero);

    front_center_pose_.header.stamp = node_->get_clock()->now();
    tf2::doTransform(front_center_pose_, current_pose, base_to_map);

    base_link_pose_.header.stamp = node_->get_clock()->now();
    tf2::doTransform(base_link_pose_, current_base_link, base_to_map);
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

  // 2. Longitudinal control
  double distance_to_goal = nav2_util::geometry_utils::euclidean_distance(current_base_link.pose.position, path_.poses[path_.poses.size() - 1].pose.position);
  double target_speed = std::min(distance_to_goal * k_speed_distance_, speed_limit_);
  double actual_speed = std::sqrt(std::pow(velocity.linear.x, 2) + std::pow(velocity.linear.y, 2));
  double speed_diff = target_speed - actual_speed;
  static double speed_diff_i = 0;
  double speed_setpoint = speed_diff * k_speed_p_ + speed_diff_i * k_speed_i_;
  speed_diff_i += speed_diff;
  if (distance_to_goal < movement_cutoff_distance_) {
    speed_setpoint = 0;
  }

  // 1b. steering angle
  double steering_angle = heading_error + std::atan2(cross_track_error * k_cte_, target_speed);
  if (steering_angle > max_steering_angle_) {
    steering_angle = max_steering_angle_;
  }
  if (steering_angle < -max_steering_angle_) {
    steering_angle = -max_steering_angle_;
  }


  // 3. Compute command velocity
  static double last_speed_setpoint = 0;
  static double last_time = node_->get_clock()->now().seconds();

  double now = node_->get_clock()->now().seconds();
  double dt = now - last_time;

  double min_speed = std::max(last_speed_setpoint - max_acceleration_ * dt, 0.0);
  double max_speed = std::min(last_speed_setpoint + max_acceleration_ * dt, speed_limit_);
  speed_setpoint = std::max(std::min(speed_setpoint, max_speed), min_speed);

  last_speed_setpoint = speed_setpoint;
  last_time = now;

  // 3b. convert to twist command
  double speed_setpoint_x = speed_setpoint * std::cos(steering_angle);
  double speed_setpoint_y = speed_setpoint * std::sin(steering_angle);
  double target_angular_rate = speed_setpoint_y / wheelbase_;

  // 4. collision checking
  while (true) {
    // recheck and reduce until collision checks pass
    auto base_to_map_projected = base_to_map;
    double dtheta = target_angular_rate * collision_lookahead_time_;
    double travel_theta = robot_angle + dtheta / 2;
    double travel_x = speed_setpoint_x * collision_lookahead_time_ * std::cos(travel_theta);
    double travel_y = speed_setpoint_x * collision_lookahead_time_ * std::sin(travel_theta);
    base_to_map_projected.transform.translation.x += travel_x;
    base_to_map_projected.transform.translation.y += travel_y;
    tf2::Quaternion q;
    q.setRPY(0, 0, tf2::getYaw(base_to_map.transform.rotation) + dtheta);
    base_to_map_projected.transform.rotation = tf2::toMsg(q);

    PoseStamped front_left_projected;
    front_left_pose_.header.stamp = node_->get_clock()->now();
    tf2::doTransform(front_left_pose_, front_left_projected, base_to_map_projected);

    PoseStamped front_right_projected;
    front_right_pose_.header.stamp = node_->get_clock()->now();
    tf2::doTransform(front_right_pose_, front_right_projected, base_to_map_projected);

    PoseStamped front_center_projected;
    front_center_pose_.header.stamp = node_->get_clock()->now();
    tf2::doTransform(front_center_pose_, front_center_projected, base_to_map_projected);

    bool left_bumper = getPoseCost(front_left_projected) > collision_costmap_value;
    bool right_bumper = getPoseCost(front_right_projected) > collision_costmap_value;
    bool center_bumper = getPoseCost(front_center_projected) > collision_costmap_value;

    if (center_bumper) {
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Frontal collision avoidance active");
      speed_setpoint_x *= collision_speed_step_factor;
    } else if ((left_bumper && target_angular_rate > 0) || (right_bumper && target_angular_rate < 0)) {
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Inside corner collision avoidance active");
      target_angular_rate *= collision_angular_rate_step_factor;
    } else if (left_bumper || right_bumper) {
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Outside corner collision avoidance active");
      target_angular_rate /= collision_angular_rate_step_factor;
    }
    else {
      break;
    }
  }


  TwistStamped cmd_vel{};
  cmd_vel.twist.linear.x = speed_setpoint_x;
  cmd_vel.twist.angular.z = target_angular_rate;

  return cmd_vel;
}

// -----------------------------------------------------------------------------
void TugController::setSpeedLimit(
  const double& speed_limit,
  const bool& percentage
)
{
  speed_limit_ = speed_limit;
}

// -----------------------------------------------------------------------------
void TugController::deactivate()
{
}

// -----------------------------------------------------------------------------
void TugController::cleanup()
{
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
  if (path.poses.size() >= 2) {
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
  } else {
    return tf2::getYaw(path.poses[path.poses.size() - 1].pose.orientation);
  }
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

double TugController::getPoseCost(PoseStamped pose) {
  unsigned int mx, my;
  if (cost_map_->getCostmap()->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    double cost = cost_map_->getCostmap()->getCost(mx, my);
    return cost;
  } else {
    return 0;
  }
}
} /* namespace tug_controller */

PLUGINLIB_EXPORT_CLASS(tug_controller::TugController, nav2_core::Controller);