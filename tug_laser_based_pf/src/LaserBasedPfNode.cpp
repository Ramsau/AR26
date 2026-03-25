#include "tug_laser_based_pf/LaserBasedPfNode.hpp"

#include <chrono>
#include <functional>
#include <queue>
#include <tf2/utils.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tug_laser_based_pf
{

// -----------------------------------------------------------------------------
LaserBasedPfNode::LaserBasedPfNode()
  : Node("tug_laser_based_pf")
{
  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_pose_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // Publisher
  particles_pub_ = create_publisher<PoseArray>("particles", 100);
  pose_with_cov_pub_ = create_publisher<PoseWithCovarianceStamped>(
    "robot_pose_with_cov",
    100
  );
  pose_pub_ = create_publisher<PoseStamped>("robot_pose", 100);
  vis_pub_ = create_publisher<Marker>("uncertainty_marker", 100);

  // Subscriber
  odom_sub_ = create_subscription<Odometry>(
    "odometry",
    10,
    std::bind(&LaserBasedPfNode::odometryCallback, this, std::placeholders::_1)
  );
  laser_sub_ = create_subscription<LaserScan>(
    "scan",
    10,
    std::bind(&LaserBasedPfNode::laserCallback, this, std::placeholders::_1)
  );

  // Service Client
  map_client_ = create_client<GetMap>("map_server/map");

  // Misc
  x_ = Eigen::MatrixXd::Zero(3, 1);
  num_partivles_ = 100;
  
  // Get static map
  RCLCPP_INFO(get_logger(), "Waiting for map service");
  if (!map_client_->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(get_logger(), "Unable to find service");
    rclcpp::shutdown();
  }

  GetMap::Request::SharedPtr req = std::make_shared<GetMap::Request>();
  map_client_->async_send_request(
    req,
    [this](rclcpp::Client<GetMap>::SharedFuture res)
    {
      RCLCPP_INFO(get_logger(), "Map received");
      RCLCPP_INFO(get_logger(), "Starting localization");
      initParticles(res.get()->map);
    }
  );

  likelihood_field_pub_ = create_publisher<OccupancyGrid>("likelihood_field", 100);
}

// -----------------------------------------------------------------------------
LaserBasedPfNode::~LaserBasedPfNode()
{
  
}

// -----------------------------------------------------------------------------
void LaserBasedPfNode::initParticles(const OccupancyGrid& map)
{
  //get max x and y values - use them to distribute your particles over the whole map
  occupancy_grid_ = map;
  max_y_position_ = static_cast<int>(map.info.height * map.info.resolution);
  max_x_position_ = static_cast<int>(map.info.width * map.info.resolution);

  // 1.) Initialize the Likelihood Field
  // calculate distance to closest object using brushfire alg
  likelihood_field_ = std::vector<double>(map.data.size(),  -1.0);
  likelihood_field_map_ = map;
  // first is the actual point, second is the closest map point
  std::queue<std::pair<Point, Point>> candidate_points;
  for (int i = 0; i < likelihood_field_.size(); i++) {
    if (map.data[i] == 100) {
      candidate_points.push({
        {i % (int)map.info.width, i / (int)map.info.width},
        {i % (int)map.info.width, i / (int)map.info.width}
      });
      likelihood_field_[i] = 0;
      likelihood_field_map_.data[i] = 0;
    }
  }
  while (!candidate_points.empty()) {
    auto current_point = candidate_points.front();
    candidate_points.pop();
    double distance = likelihood_field_[current_point.first.x + current_point.first.y * (int)map.info.width];
    assert(distance != -1.0);
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        int new_x = current_point.first.x + dx;
        int new_y = current_point.first.y + dy;
        if (new_x >= 0 && new_x < (int)map.info.width && new_y >= 0 && new_y < (int)map.info.height) {
          int new_index = new_y * (int)map.info.width + new_x;
          double new_dist = std::sqrt(
            std::pow(current_point.second.x - new_x, 2) +
            std::pow(current_point.second.y - new_y, 2)
          );
          if (likelihood_field_[new_index] == -1.0 || new_dist < likelihood_field_[new_index]) {
            likelihood_field_[new_index] = new_dist;
            likelihood_field_map_.data[new_index] = new_dist;
            candidate_points.push({
              {new_x, new_y},
              current_point.second
            });
          }
        }
      }
    }
  }
  // convert distance to likelihood
  for (int i = 0; i < likelihood_field_.size(); i++) {
    likelihood_field_[i] = std::exp(
                             -std::pow(likelihood_field_[i], 2) / (std::pow(sigma_hit, 2) * 2)
                           ) / std::sqrt(2 * M_PI * sigma_hit);
    likelihood_field_map_.data[i] = likelihood_field_[i] * 100 * std::sqrt(2 * M_PI * sigma_hit);
  }
  likelihood_field_pub_->publish(likelihood_field_map_);

  
  // 2.) Initialize the Sample Set
  for (int i = 0; i < num_partivles_; i++) {
    particles_.emplace_back();
    particles_.back().updatePose(
      occupancy_grid_.info.origin.position.x + max_x_position_ * randomDouble(),
      occupancy_grid_.info.origin.position.y + max_y_position_ * randomDouble(),
      2 * M_PI * randomDouble()
    );
    if (std::isnan(particles_.back().getX() + particles_.back().getY() + particles_.back().getTheta())) {
      RCLCPP_ERROR_STREAM(get_logger(), "particle x:" << particles_.back().getX() << " y:" << particles_.back().getY() << " theta:" << particles_.back().getTheta());
      while (true);
    }
    particles_.back().updateWeight(1.0);
  }

  //normalize weight of particles
  normalizeParticleWeights();

  particles_initialized_ = true;
}

// -----------------------------------------------------------------------------
void LaserBasedPfNode::updateOdometry(const Odometry& odom)
{
  // move particles the same way the robot moved
  static bool first_call = true;

  if (first_call)
  {
    last_odometry_ = odom;
    
    /*
    tf2::Quaternion q;
    tf2::fromMsg(odom.pose.pose.orientation, q);

    tf2::Matrix3x3 m(q);
    double roll;
    double pitch;
    double yaw;
    m.getRPY(roll, pitch, yaw);
    resetLocalization(odom.pose.pose.position.x, odom.pose.pose.position.y, yaw);
    */
    
    updateLocalization(x_, particles_);
    first_call = false;
    return;
  }

  if (!particles_initialized_) {
    RCLCPP_ERROR_STREAM(get_logger(), "particles not initialized");
    return;
  }


  // 1. Enter your odometry update for each particle

  // global variable last_odometry_ contains the last odometry position estimation (ROS Odometry Messasge)
  // local variable odometry contains the current odometry position estimation (ROS Odometry Messasge)
  double delta_x = odom.pose.pose.position.x - last_odometry_.pose.pose.position.x;
  double delta_y = odom.pose.pose.position.y - last_odometry_.pose.pose.position.y;
  double delta_trans = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));

  double delta_theta_1;
  double delta_theta_2;
  double theta_travel = 0;
  if (delta_trans < travel_epsilon) {
    delta_theta_1 = 0;
    delta_theta_2 = normalizeAngle(tf2::getYaw(odom.pose.pose.orientation) - tf2::getYaw(last_odometry_.pose.pose.orientation));
  } else {
    theta_travel = normalizeAngle(std::atan2(delta_y, delta_x));
    delta_theta_1 = normalizeAngle(theta_travel - tf2::getYaw(last_odometry_.pose.pose.orientation));
    delta_theta_2 = normalizeAngle(tf2::getYaw(odom.pose.pose.orientation) - theta_travel);

    // driving backwards
    if (std::abs(delta_theta_1) > M_PI / 2.0 && std::abs(delta_theta_2) > M_PI / 2.0) {
      delta_trans = -delta_trans;
      delta_theta_1 = normalizeAngle(delta_theta_1 + M_PI);
      delta_theta_2 = normalizeAngle(delta_theta_2 + M_PI);
    }
  }

  RCLCPP_DEBUG_STREAM(get_logger(), "dx:" << delta_x << " dy:" << delta_y << " thtravel:" << theta_travel << " orientation:" << tf2::getYaw(odom.pose.pose.orientation) << " lastOr:" << tf2::getYaw(last_odometry_.pose.pose.orientation) << " dtheta1:" << delta_theta_1 << " dtheta2:" << delta_theta_2);


  for (auto &particle : particles_) {
    double delta_theta_1_noise = delta_theta_1 + sampleNormalDistribution(alpha_1 * std::abs(delta_theta_1) + alpha_2 * delta_trans);
    double delta_trans_noise = delta_trans + sampleNormalDistribution(alpha_3 * std::abs(delta_trans) + alpha_4 * (std::abs(delta_theta_1) + std::abs(delta_theta_2)));
    double delta_theta_2_noise = delta_theta_2 + sampleNormalDistribution(alpha_1 * std::abs(delta_theta_2) + alpha_2 * delta_trans);
    if (std::isnan(particle.getX() + particle.getY() + particle.getTheta())) {
      RCLCPP_ERROR_STREAM(get_logger(), "particle x:" << particle.getX() << " y:" << particle.getY() << " theta:" << particle.getTheta());
      while (true);
    }
    particle.updatePose(
      particle.getX() + delta_trans_noise * std::cos(particle.getTheta() + delta_theta_1_noise),
      particle.getY() + delta_trans_noise * std::sin(particle.getTheta() + delta_theta_1_noise),
      particle.getTheta() + delta_theta_1_noise + delta_theta_2_noise);
    if (std::isnan(particle.getX() + particle.getY() + particle.getTheta())) {
      RCLCPP_ERROR_STREAM(get_logger(), "particle x:" << particle.getX() << " y:" << particle.getY() << " theta:" << particle.getTheta());
      while (true);
    }
  }


  // Keep This - reports your update
  updateLocalization(x_, particles_);
  last_odometry_ = odom;
}

// -----------------------------------------------------------------------------
void LaserBasedPfNode::updateLaser(const LaserScan& scan)
{
  // 1. Compute the pose of the virutal laser for each particle
  // - keep in mind that the laser is not positioned at the particle
  // 2. Derive the propbality of a laser measurement using the likelihood field
	// 3. Compoute the probability of the particles

  if (!particles_initialized_) {
    RCLCPP_ERROR_STREAM(get_logger(), "particles not initialized");
    return;
  }

  for (auto &particle : particles_) {
    double weight = 0;
    for (int i = 0; i < scan.ranges.size(); i++) {
      double range = scan.ranges[i];
      double range_angle = particle.getTheta() + scan.angle_min + scan.angle_increment * i;
      double p_max, p_miss, p_hit;
      if (range == std::numeric_limits<double>::infinity()) {
        p_max = 1;
        p_miss = 0;
        p_hit = 0;
      } else {
        p_max = 0;
        p_miss = 1;

        double hit_x = particle.getX() + range * std::cos(range_angle);
        double hit_y = particle.getY() + range * std::sin(range_angle);
        double hit_x_px = static_cast<int>(
          (hit_x - likelihood_field_map_.info.origin.position.x) / likelihood_field_map_.info.resolution
        );
        double hit_y_px = static_cast<int>(
          (hit_y - likelihood_field_map_.info.origin.position.y) / likelihood_field_map_.info.resolution
        );
        if (hit_x_px >= 0 && hit_x_px < likelihood_field_map_.info.width && hit_y_px >= 0 && hit_y_px < likelihood_field_map_.info.height) {
          p_hit = likelihood_field_map_.data[hit_y_px * likelihood_field_map_.info.width + hit_x_px];
        } else {
          p_hit = 0;
        }
      }
      weight += p_hit * alpha_hit + p_miss * alpha_miss + p_max * alpha_max;
    }
    RCLCPP_DEBUG_STREAM(get_logger(), "weight:" << weight);
    particle.updateWeight(weight);
  }
   
   
  // normalize your weights
  normalizeParticleWeights();
  // do resampling
  resamplingParticles();

  // Keep This - reports your update
  updateLocalization(x_, particles_);
}

// -----------------------------------------------------------------------------
void LaserBasedPfNode::normalizeParticleWeights()
{
  // Normalize the particles
  double sum = 0;
  for (size_t i = 0; i < particles_.size(); i++) {
    sum += particles_.at(i).getWeight();
  }
  for (size_t i = 0; i < particles_.size(); i++) {
    particles_.at(i).updateWeight(particles_.at(i).getWeight() / sum);
  }
}

// -----------------------------------------------------------------------------
void LaserBasedPfNode::resamplingParticles()
{
  // Resample the particles
  std::vector<Particle> new_particles{};
  double threshold = particles_.at(0).getWeight();
  double i = 0;
  double u = randomDouble() / num_partivles_;
  for (int j = 0; j < particles_.size(); j++) {
    while (u > threshold) {
      i++;
      threshold += particles_.at(i).getWeight();
    }

    new_particles.push_back(particles_.at(i));
    u += 1.0 / num_partivles_;
  }
  RCLCPP_DEBUG_STREAM(get_logger(), "old sample count:" << particles_.size() << " new sample count:" << new_particles.size());
  assert(new_particles.size() == num_partivles_);
  particles_ = new_particles;
  for (auto &particle : particles_) {
    particle.updateWeight(1.0 / num_partivles_);
    if (std::isnan(particle.getX() + particle.getY() + particle.getTheta())) {
      RCLCPP_ERROR_STREAM(get_logger(), "particle x:" << particle.getX() << " y:" << particle.getY() << " theta:" << particle.getTheta());
      while (true);
    }
  }

}

// -----------------------------------------------------------------------------
void LaserBasedPfNode::publishPose(
  Eigen::MatrixXd& x,
  const std::vector<Particle>& particles
)
{
  //calculate mean of given particles
  double x_mean = 0;
  double y_mean = 0;
  double yaw_mean = 0;

  // TODO Calculate the robot pose from the particles

  x(0,0) = x_mean;
  x(1,0) = y_mean;
  x(2,0) = yaw_mean;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_mean);

  tf2::Transform transform;
  transform.setOrigin( { x_mean, y_mean, 0.0 } );
  transform.setRotation(q);

  TransformStamped stamped_tf;
  stamped_tf.header.stamp = get_clock()->now();
  stamped_tf.header.frame_id = "map";
  stamped_tf.child_frame_id = "base_link_pf";
  stamped_tf.transform = tf2::toMsg(transform);

  tf_pose_broadcaster_->sendTransform(stamped_tf);

  //calculate covariance matrix
  double standard_deviation_x = 0;
  double standard_deviation_y = 0;
  double standard_deviation_theta = 0;

  //todo check if uncertainty possible
  for (size_t i = 0; i < particles.size(); i++)
  {
    standard_deviation_x += std::pow(particles[i].getX() - x_mean, 2);
    standard_deviation_y += std::pow(particles[i].getY() - y_mean, 2);
    standard_deviation_theta += std::pow(particles[i].getTheta() - yaw_mean, 2);
  }

  standard_deviation_theta = std::sqrt(standard_deviation_theta);
  standard_deviation_x = std::sqrt(standard_deviation_x);
  standard_deviation_y = std::sqrt(standard_deviation_y);
  standard_deviation_theta /= static_cast<double>(particles.size() - 1);
  standard_deviation_x /= static_cast<double>(particles.size() - 1);
  standard_deviation_y /= static_cast<double>(particles.size() - 1);

  //need to bound it otherwise calc of uncertainty marker doesn't work
  double thresh = 0.0000001;
  if(standard_deviation_theta < thresh)
      standard_deviation_theta = thresh;

  if(standard_deviation_x < thresh)
      standard_deviation_x = thresh;

  if(standard_deviation_y < thresh)
      standard_deviation_y = thresh;

  //put in right msg
  PoseWithCovarianceStamped pose_with_cov;
  pose_with_cov.header.frame_id = "map";
  pose_with_cov.header.stamp = get_clock()->now();
  pose_with_cov.pose.pose.position.x = x_mean;
  pose_with_cov.pose.pose.position.y = y_mean;
  pose_with_cov.pose.pose.position.z = 0;
  pose_with_cov.pose.pose.orientation.w = q.getW();
  pose_with_cov.pose.pose.orientation.x = q.getX();
  pose_with_cov.pose.pose.orientation.y = q.getY();
  pose_with_cov.pose.pose.orientation.z = q.getZ();

  pose_with_cov.pose.covariance[0] = std::pow(standard_deviation_x,2);
  pose_with_cov.pose.covariance[7] = std::pow(standard_deviation_y,2);
  pose_with_cov.pose.covariance[35] = std::pow(standard_deviation_theta,2);
  pose_with_cov_pub_->publish(pose_with_cov);

  // Uncertainty Visualization
  Eigen::Matrix2f uncertainty_mat;
  uncertainty_mat(0,0) = standard_deviation_x * 100.0;
  uncertainty_mat(0,1) = thresh;
  uncertainty_mat(1,0) = thresh;
  uncertainty_mat(1,1) = standard_deviation_y * 100.0;

  Eigen::Vector2f uncertainty_position;
  uncertainty_position(0) = x(0,0);
  uncertainty_position(1) = x(1,0);

  Marker uncertainly_marker;
  generateUncertaintyMarker(uncertainly_marker, uncertainty_mat, uncertainty_position);
  vis_pub_->publish(uncertainly_marker);
}

// -----------------------------------------------------------------------------
void LaserBasedPfNode::generateUncertaintyMarker(
  Marker& marker,
  const Eigen::Matrix2f& uncertainly_mat,
  const Eigen::Vector2f& position
)
{
  Eigen::EigenSolver<Eigen::Matrix2f> solver(uncertainly_mat);
  Eigen::VectorXf uncertainty_eigenvalues = solver.eigenvalues().real();
  Eigen::MatrixXf uncertainty_eigenvectors = solver.eigenvectors().real();

  double phi_ellipse = std::atan2(uncertainty_eigenvectors(0,1), uncertainty_eigenvectors(0,0));

  marker.header.frame_id = "map";
  marker.header.stamp = get_clock()->now();
  marker.ns = "ellipses";
  marker.id = 0;
  marker.type = Marker::CYLINDER;
  marker.action = Marker::ADD;

  Pose ellipse_pose;
  ellipse_pose.position.x = position(0);
  ellipse_pose.position.y = position(1);
  ellipse_pose.position.z = 0;

  tf2::Quaternion tf_quat;
  tf_quat.setRPY(0, 0, phi_ellipse);
  ellipse_pose.orientation = tf2::toMsg(tf_quat);

  marker.pose = ellipse_pose;

  // eigenvalue of uncertainty matrix is the square of the semi-major/minor of the ellipse;
  // 2.447*sigma => 95% area
  marker.scale.x = 2.447 * 2.0 * std::sqrt(uncertainty_eigenvalues(0));
  marker.scale.y = 2.447 * 2.0 * std::sqrt(uncertainty_eigenvalues(1));
  marker.scale.z = 0.1;
  marker.color.a = 0.2;
  marker.color.r = 0.9;
  marker.color.g = 0.0;
  marker.color.b = 0.3;
}

// -----------------------------------------------------------------------------
void LaserBasedPfNode::publishParticles(const std::vector<Particle>& particles)
{
  PoseArray array;
  array.poses = getParticlePositions(particles);
  array.header.frame_id = "map";
  array.header.stamp = get_clock()->now();

  particles_pub_->publish(array);
}

// -----------------------------------------------------------------------------
std::vector<LaserBasedPfNode::Pose> LaserBasedPfNode::getParticlePositions(
  const std::vector<Particle>& particles
)
{
  std::vector<Pose> positions;

  for(size_t i = 0; i < particles.size(); i++) {
    positions.push_back(particles[i].getPose());
  }

  return positions;
}

// ----------------------------------------------------------------------------
void LaserBasedPfNode::updateLocalization(
  Eigen::MatrixXd& x,
  const std::vector<Particle>& particles
)
{
  //visualisation of pose
  publishPose(x, particles);

  //visualization of particles
  publishParticles(particles);
}

// -----------------------------------------------------------------------------
double LaserBasedPfNode::probNormalDistribution(double a, double variance)
{
  if (variance == 0)
    return a;

  return (1.0 / (std::sqrt(2.0 * M_PI * variance)))
    * std::exp(-0.5 * std::pow(a, 2.0) / variance);
}

// -----------------------------------------------------------------------------
double LaserBasedPfNode::sampleNormalDistribution(double variance)
{
  constexpr double scaling_factor = 1000.0;

  if (variance <= (1.0 / scaling_factor))
    return 0;

  double sum = 0.0;
  int border = std::sqrt(variance) * static_cast<int>(scaling_factor);

  for (int i = 0; i < 12; i++)
    sum += std::rand() % (2 * border) - border;

  return sum * 0.5 / scaling_factor;
}

// -----------------------------------------------------------------------------
double LaserBasedPfNode::randomDouble() {
  return static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
}
  
// -----------------------------------------------------------------------------
double LaserBasedPfNode::normalizeAngle(double angle) {
  angle = fmod(angle + M_PI, 2 * M_PI);
  if (angle < 0) {
    angle += 2 * M_PI;
  }
  return angle - M_PI;
}

// -----------------------------------------------------------------------------
void LaserBasedPfNode::resetLocalization(double x, double y, double theta)
{
  x_(0,0) = x;
  x_(1,0) = y;
  x_(2,0) = theta;

  //distribute particles around true pose
  constexpr double scale_factor = 1000.0;

  int x_range = static_cast<int>(max_x_position_ / 10.0 * scale_factor);
  int y_range = static_cast<int>(max_y_position_ / 10.0 * scale_factor);
  int theta_range = static_cast<int>(M_PI / 4.0 * scale_factor);

  for(size_t i = 0; i < particles_.size(); i++)
  {
    double new_x = x + (std::rand() % x_range - static_cast<int>(x_range/2.0) )
      / scale_factor;
    double new_y = y + (std::rand() % y_range - static_cast<int>(y_range/2.0) )
      / scale_factor;
    double new_theta  = theta + (std::rand() % theta_range
      - static_cast<int>(theta_range / 2.0)) / scale_factor;

    particles_.at(i).updatePose(new_x, new_y, new_theta);
    if (std::isnan(particles_.at(i).getX() + particles_.at(i).getY() + particles_.at(i).getTheta())) {
      RCLCPP_ERROR(get_logger(), "NaN in particle");
      while (true);
    }
    particles_.at(i).updateWeight(1.0);
  }
}

// -----------------------------------------------------------------------------
void LaserBasedPfNode::odometryCallback(const Odometry::ConstSharedPtr& msg)
{
  data_lock_.lock();
  updateOdometry(*msg);
  data_lock_.unlock();
}

// -----------------------------------------------------------------------------
void LaserBasedPfNode::laserCallback(const LaserScan::ConstSharedPtr& msg)
{
  data_lock_.lock();
  updateLaser(*msg);
  updateLocalization(x_, particles_);
  data_lock_.unlock();
}


// -----------------------------------------------------------------------------
void LaserBasedPfNode::initialPoseCallback(
  const PoseWithCovarianceStamped::ConstSharedPtr& msg
)
{
  data_lock_.lock();
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);

  tf2::Matrix3x3 m(q);
  double roll;
  double pitch;
  double yaw;
  m.getRPY(roll, pitch, yaw);

  RCLCPP_INFO_STREAM(
    get_logger(),
    "initalPoseCallback x=" << x <<", y=" << y <<", theta=" << yaw
  );
  resetLocalization(x, y, yaw);
  data_lock_.unlock();
}

} /* namespace tug_laser_based_pf */