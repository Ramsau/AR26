#include "tug_g2o_based_mapping/G2OBasedMapping.hpp"

#include <chrono>
#include <functional>
#include <mutex>

#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tug_g2o_based_mapping
{

// -----------------------------------------------------------------------------
G2OBasedMapping::G2OBasedMapping()
  : Node("tug_g2o_based_mapping")
{
  // TF2
  transform_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *transform_buffer_
  );

  transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    *this
  );

  // Map publish timer
  map_publish_timer_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive
  );

  map_publish_timer_ = rclcpp::create_timer(
    this,
    get_clock(),
    std::chrono::milliseconds(1000), // Update rate may be modified
    std::bind(&G2OBasedMapping::publishMap, this),
    map_publish_timer_callback_group_
  );

  // Publisher
  map_publisher_ = create_publisher<OccupancyGrid>("map", 10);
  graph_cloud_publisher_ = create_publisher<PointCloud>("graph_cloud", 10);
  robot_pose_marker_publisher_ = create_publisher<MarkerArray>(
    "robot_pose_marker",
    10
  );

  fiducials_observed_marker_publisher_ = create_publisher<MarkerArray>(
    "fiducials_observed_marker",
    10
  );

  graph_edges_publisher_ = create_publisher<Marker>("graph_edges", 10);
  old_fiducials_observed_marker_publisher_ = create_publisher<MarkerArray>(
    "old_fiducials_observed_marker",
    10
  );

  // Subscriber
  odometry_subscriber_ = create_subscription<Odometry>(
    "odometry",
    100,
    std::bind(&G2OBasedMapping::odomCallback, this, std::placeholders::_1)
  );

  laser_scan_subscriber_ = create_subscription<LaserScan>(
    "scan",
    100,
    std::bind(&G2OBasedMapping::laserCallback, this, std::placeholders::_1)
  );

  initial_pose_subscriber_ = create_subscription<PoseWithCovarianceStamped>(
    "initialpose",
    100,
    std::bind(
      &G2OBasedMapping::initialPoseCallback,
      this,
      std::placeholders::_1
    )
  );

  // Messages
  graph_map_.header.frame_id = "map";
  graph_map_.info.map_load_time = get_clock()->now();
  graph_map_.info.origin.position.x = -15.0;
  graph_map_.info.origin.position.y = -15.0;
  graph_map_.info.resolution = 0.05F;
  graph_map_.info.width = 600;
  graph_map_.info.height = 600;

  // Matrices
  x_ = Eigen::MatrixXd::Zero(3, 1);

  // Solver
  using SLAMLinearSolver 
    = g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>;
  
  std::unique_ptr<SLAMLinearSolver> solver
    = std::make_unique<SLAMLinearSolver>();

  solver->setBlockOrdering(false);

  optimization_algorithm_ = new g2o::OptimizationAlgorithmGaussNewton(
    std::make_unique<g2o::BlockSolverX>(std::move(solver))
  );

  graph_.setAlgorithm(optimization_algorithm_);

  // Init
  init(0.0, 0.0, 0.0);

  // TODO
  // find appropriate parameters
  double x_noise = 1;
  double y_noise = 1;
  double rot_noise = 1;    //rad
  double landmark_x_noise = 1;
  double landmark_y_noise = 1;

  odom_noise_.fill(0.0);
  odom_noise_(0, 0) = 1 / (x_noise * x_noise);
  odom_noise_(1, 1) = 1 / (y_noise * y_noise);
  odom_noise_(2, 2) = 1 / (rot_noise * rot_noise);

  laser_noise_.fill(0.0);
  laser_noise_(0, 0) = 1;
  laser_noise_(1, 1) = 1;
  laser_noise_(2, 2) = 1;

  landmark_noise_.fill(0.0);
  landmark_noise_(0, 0) = 1 / (landmark_x_noise * landmark_x_noise);
  landmark_noise_(1, 1) = 1 / (landmark_y_noise * landmark_y_noise);
}

// -----------------------------------------------------------------------------
G2OBasedMapping::~G2OBasedMapping()
{
  delete optimization_algorithm_;
  delete laser_params_;
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::updateOdometry(const Odometry::ConstSharedPtr& odom)
{
  if (reset_)
  {
    last_odometry_ = *odom;

    updateLocalization();

    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;
    double yaw = yawFromQuaternion(odom->pose.pose.orientation);

    init(x, y, yaw);

    reset_ = false;
    valid_ = false;
    return;
  }

  // TODO
  // 1. Enter your odometry update here
  // 2. Keep track of the odometry updates to the robot position

  // global variable last_odometry_ contains the last odometry position estimation (ROS Odometry Messasge)
  // local variable oodom contains the current odometry position estimation (ROS Odometry Messasge)

  // global variable x_ holds your position (Eigen vector of size 3 [x,y,theta])

  // Keep This - reports your update
  last_odometry_ = *odom;
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::updateLaser(const LaserScan::ConstSharedPtr& laser)
{
  if (!laser_params_ || graph_.vertices().size() == 0)
    {
        // first laser update
        laser_params_ = new g2o::LaserParameters(
          laser->ranges.size(),
          laser->angle_min,
          laser->angle_increment,
          laser->range_max
        );

        addLaserVertex(x_(0), x_(1), x_(2), *laser, last_id_, true);
        return;
    }

    // TODO
    // 1. Enter your laser scan update here
    // 2. Build up the pose graph by adding odometry and laser edges
    // 3. Check for loop closures
    // 4. Optimize the graph


    // Keep This - reports your update
    updateLocalization();
    visualizeRobotPoses();
    // Keep This - if you like to visualize your map (collected laser scans in the graph)
    visualizeLaserScans();
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::init(double x, double y, double theta)
{
  x_(0, 0) = x;
  x_(1, 0) = y;
  x_(2, 0) = theta;

  graph_.clear();
  edge_set_.clear();
  vertex_set_.clear();
  seen_landmarks_.clear();
  robot_pose_ids_.clear();
  robot_landmark_edge_ids_.clear();
  laser_edge_ids_.clear();
  
  valid_ = false;
  reset_ = true;
  robot_pose_set_ = true;
  first_opt_ = true;

  min_to_optimize_ = 4;
  last_id_ = 30;

  visualizeOldLandmarks();
  visualizeLandmarks();
  visualizeRobotPoses();
  visualizeEdges();
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::addOdomVertex(
  double x,
  double y,
  double theta,
  int id,
  bool first
)
{
  g2o::SE2 pose(x, y, theta);
  g2o::VertexSE2* vertex = new g2o::VertexSE2;
  vertex->setId(id);
  vertex->setEstimate(pose);

  graph_.addVertex(vertex);
  vertex_set_.insert(vertex);
  robot_pose_ids_.push_back(id);

  if(first)
    vertex->setFixed(true);
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::addLaserVertex(
  double x,
  double y,
  double theta,
  LaserScan scan,
  int id,
  bool first
)
{
  g2o::SE2 pose(x, y, theta);
  g2o::VertexSE2* vertex = new g2o::VertexSE2;
  vertex->setId(id);
  vertex->setEstimate(pose);

  g2o::RawLaser* rl = new g2o::RawLaser();
  rl->setLaserParams(*laser_params_);

  std::vector<double> r;
  std::vector<float>::iterator it = scan.ranges.begin();

  r.assign(it, scan.ranges.end());
  rl->setRanges(r);
  vertex->addUserData(rl);
  graph_.addVertex(vertex);
  vertex_set_.insert(vertex);
  robot_pose_ids_.push_back(id);

  if(first)
    vertex->setFixed(true);
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::addLaserEdge(
  int id1,
  int id2,
  double x,
  double y,
  double yaw,
  Eigen::Matrix3d noise
)
{
  g2o::EdgeSE2* edge = new g2o::EdgeSE2;
  edge->vertices()[0] = graph_.vertex(id1);
  edge->vertices()[1] = graph_.vertex(id2);
  edge->setMeasurement(g2o::SE2(x,y,yaw));
  edge->setInformation(noise);

  laser_edge_ids_.push_back(std::pair<int, int>(id1, id2));

  graph_.addEdge(edge);
  edge_set_.insert(edge);

  RCLCPP_INFO_STREAM(get_logger(), "added laser edge: " << id1 << " - " << id2);
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::addOdomEdge(int id1, int id2)
{
  std::vector<double> data1,data2;

  graph_.vertex(id1)->getEstimateData(data1);
  graph_.vertex(id2)->getEstimateData(data2);

  g2o::SE2 vertex1(data1[0], data1[1], data1[2]);
  g2o::SE2 vertex2(data2[0], data2[1], data2[2]);

  g2o::SE2 transform = vertex1.inverse() * vertex2;
  g2o::EdgeSE2* edge = new g2o::EdgeSE2;
  edge->vertices()[0] = graph_.vertex(id1);
  edge->vertices()[1] = graph_.vertex(id2);
  edge->setMeasurement(transform);
  edge->setInformation(odom_noise_);

  graph_.addEdge(edge);
  edge_set_.insert(edge);

  RCLCPP_INFO_STREAM(
    get_logger(),
    "added odometry edge: " << id1 << " - " << id2
  );
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::addLandmarkVertex(double x, double y, int id)
{
  if(graph_.vertex(id))
    return;

  Eigen::Vector2d pos(x, y);
  g2o::VertexPointXY *vertex = new g2o::VertexPointXY;
  vertex->setId(id);
  vertex->setEstimate(pos);

  seen_landmarks_.push_back(id);
  graph_.addVertex(vertex);
  vertex_set_.insert(vertex);
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::addLandmarkEdge(int id1, int id2, double x, double y)
{
  std::vector<double> data;
  graph_.vertex(id1)->getEstimateData(data);

  g2o::SE2 vertex1(data[0], data[1], data[2]);
  Eigen::Vector2d vertex2(x, y);
  Eigen::Vector2d measurement;
  measurement = vertex1.inverse() * vertex2;

  g2o::EdgeSE2PointXY* landmark_edge =  new g2o::EdgeSE2PointXY;
  landmark_edge->vertices()[0] = graph_.vertex(id1);
  landmark_edge->vertices()[1] = graph_.vertex(id2);
  landmark_edge->setMeasurement(measurement);
  landmark_edge->setInformation(landmark_noise_);

  graph_.addEdge(landmark_edge);
  edge_set_.insert(landmark_edge);
  robot_landmark_edge_ids_.push_back(std::pair<int, int>(id1, id2));

  RCLCPP_INFO_STREAM(
    get_logger(),
    "added landmark edge: " << id1 << " - " << id2
  );
}

// ----------------------------------------------------------------------------
void G2OBasedMapping::optimizeGraph()
{
  graph_.save("state_before.g2o");
  graph_.setVerbose(true);
  visualizeOldLandmarks();

  RCLCPP_INFO_STREAM(get_logger(), "Optimizing");

  if(first_opt_)
  {
    if(!graph_.initializeOptimization())
      RCLCPP_ERROR_STREAM(get_logger(), "FAILED initializeOptimization");
  }

  else if(!graph_.updateInitialization(vertex_set_, edge_set_))
    RCLCPP_ERROR_STREAM(get_logger(), "FAILED updateInitialization");

  int iterations = 10;
  graph_.optimize(iterations, !first_opt_);
  graph_.save("state_after.g2o");

  first_opt_ = false;
  vertex_set_.clear();
  edge_set_.clear();

  setRobotToVertex(robot_pose_ids_.back());
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::setRobotToVertex(int id)
{
  std::vector<double> data;
  graph_.vertex(id)->getEstimateData(data);

  x_(0, 0) = data[0];
  x_(1, 0) = data[1];
  x_(2, 0) = data[2];

  updateLocalization();
}

// -----------------------------------------------------------------------------
G2OBasedMapping::LaserScan G2OBasedMapping::rawLasertoLaserScanMsg(
  g2o::RawLaser rawlaser
)
{
  LaserScan msg;
  msg.header.frame_id = "base_laser_link";
  msg.angle_min = rawlaser.laserParams().firstBeamAngle;
  msg.angle_increment = rawlaser.laserParams().angularStep;
  msg.range_min = 0;
  msg.range_max = rawlaser.laserParams().maxRange;

  std::vector<double>::const_iterator it = rawlaser.ranges().begin();
  msg.ranges.assign(it, rawlaser.ranges().end());

  return msg;
}

// ----------------------------------------------------------------------------
void G2OBasedMapping::visualizeLaserScans()
{
  PointCloud graph_cloud;
  graph_cloud.header.frame_id = "map";
  graph_cloud.header.stamp = get_clock()->now();
  
  for(size_t j = 0; j < robot_pose_ids_.size(); j++)
  {
    std::vector<double> data;
    graph_.vertex(robot_pose_ids_[j])->getEstimateData(data);

    g2o::OptimizableGraph::Data* d
      = graph_.vertex(robot_pose_ids_[j])->userData();

    g2o::RawLaser* rawLaser = dynamic_cast<g2o::RawLaser*>(d);

    if (rawLaser)
    {
      float angle = rawLaser->laserParams().firstBeamAngle;
      for(
        std::vector<double>::const_iterator i = rawLaser->ranges().begin();
        i != rawLaser->ranges().end();
        i++
      )
      {
        Point32 p;
        float x = *i * cos(angle);
        float y = *i * sin(angle);

        p.x = data[0] + x * cos(data[2]) - y * sin(data[2]);
        p.y = data[1] + x * sin(data[2]) + y * cos(data[2]);
        p.z = 0;
        angle += rawLaser->laserParams().angularStep;
        graph_cloud.points.push_back(p);
      }
    }
  }

  graph_cloud_publisher_->publish(graph_cloud);
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::visualizeRobotPoses()
{
  Marker marker;
  MarkerArray marker_array;

  marker.header.frame_id = "map";
  marker.header.stamp = get_clock()->now();
  marker.ns = "robot_poses";
  marker.pose.position.z = 0.0;
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 0.5;
  marker.color.r = 0.1;
  marker.color.g = 0.1;
  marker.color.b = 0.9;

  for(size_t j = 0; j < robot_pose_ids_.size(); j++)
  {
    // Sphere Marker
    std::vector<double> data;
    graph_.vertex(robot_pose_ids_[j])->getEstimateData(data);
    marker.pose.position.x = data[0];
    marker.pose.position.y = data[1];
    marker.id = robot_pose_ids_[j];
    marker_array.markers.push_back(marker);
  }

  robot_pose_marker_publisher_->publish(marker_array);

  visualizeEdges();
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::visualizeLandmarks()
{
  Marker marker;
  Marker marker_text;;
  MarkerArray marker_array;
  MarkerArray marker_array_text;

  marker.header.frame_id = "map";
  marker.header.stamp = get_clock()->now();
  marker.pose.position.z = 0.0;
  marker.ns = "observed_fiducials";
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 0.6;
  marker.color.a = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 0.3;
  marker.color.b = 0.0;
  
  marker_text.header = marker.header;
  marker_text.ns = "observed_fiducials_text";
  marker_text.type = Marker::TEXT_VIEW_FACING;
  marker_text.action = Marker::ADD;
  marker_text.scale.z = 0.6 * 0.85;
  marker_text.color.a = 0.7;
  marker_text.color.r = 0.0;
  marker_text.color.g = 0.0;
  marker_text.color.b = 0.0;
  
  for(size_t j = 0; j < seen_landmarks_.size(); j++)
  {
    // Sphere Marker
    std::vector<double> data;
    graph_.vertex(seen_landmarks_[j])->getEstimateData(data);

    marker.pose.position.x = data[0];
    marker.pose.position.y = data[1];
    marker.id = seen_landmarks_[j];
    marker_array.markers.push_back(marker);

    // Text Marker
    marker_text.pose.position = marker.pose.position;
    marker_text.id = seen_landmarks_[j];
    marker_text.text = marker_text.id;
    marker_array_text.markers.push_back(marker_text);
  }

  fiducials_observed_marker_publisher_->publish(marker_array);
  fiducials_observed_marker_publisher_->publish(marker_array_text);

  visualizeEdges();
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::visualizeEdges()
{
  Marker marker;
  MarkerArray marker_array;

  marker.header.frame_id = "map";
  marker.header.stamp = get_clock()->now();
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 0.5;
  marker.color.r = 0.9;
  marker.color.g = 0.1;
  marker.color.b = 0.1;
  marker.id = 0;
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.ns = "edges";

  Point p;
  p.z = 0;

  std::vector<double> data;

  for(size_t j = 0; j < robot_pose_ids_.size(); j++)
  {
    graph_.vertex(robot_pose_ids_[j])->getEstimateData(data);

    p.x = data[0];
    p.y = data[1];
    marker.points.push_back(p);
  }

  graph_edges_publisher_->publish(marker);

  marker.points.clear();
  marker.id = 1;
  marker.type = Marker::LINE_LIST;

  for(size_t j = 0; j < robot_landmark_edge_ids_.size(); j++)
  {
    graph_.vertex(robot_landmark_edge_ids_[j].first)->getEstimateData(data);
    p.x = data[0];
    p.y = data[1];
    marker.points.push_back(p);

    graph_.vertex(robot_landmark_edge_ids_[j].second)->getEstimateData(data);
    p.x = data[0];
    p.y = data[1];
    marker.points.push_back(p);
  }

  for(size_t j = 0; j < laser_edge_ids_.size(); j++)
  {
    graph_.vertex(laser_edge_ids_[j].first)->getEstimateData(data);
    p.x = data[0];
    p.y = data[1];
    marker.points.push_back(p);

    graph_.vertex(laser_edge_ids_[j].second)->getEstimateData(data);
    p.x = data[0];
    p.y = data[1];
    marker.points.push_back(p);
  }

  graph_edges_publisher_->publish(marker);
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::visualizeOldLandmarks()
{
  Marker marker;
  Marker marker_text;;
  MarkerArray marker_array;
  MarkerArray marker_array_text;

  marker.header.frame_id = "map";
  marker.header.stamp = get_clock()->now();
  marker.pose.position.z = 0.0;
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 0.6;
  marker.color.a = 0.5;
  marker.color.r = 0.1;
  marker.color.g = 0.1;
  marker.color.b = 0.8;
  marker.ns = "old_observed_fiducials";

  marker_text.header = marker.header;
  marker_text.pose.position = marker.pose.position;
  marker_text.type = Marker::TEXT_VIEW_FACING;
  marker_text.action = Marker::ADD;
  marker_text.scale.z = 0.6*0.85;
  marker_text.color.a = 0.7;
  marker_text.color.r = 0.0;
  marker_text.color.g = 0.0;
  marker_text.color.b = 0.0;
  marker_text.ns = "old_observed_fiducials_text";

  for(size_t j = 0; j < seen_landmarks_.size(); j++)
  {
    // Sphere Marker
    std::vector<double> data;
    graph_.vertex(seen_landmarks_[j])->getEstimateData(data);

    marker.pose.position.x = data[0];
    marker.pose.position.y = data[1];
    marker.id = seen_landmarks_[j];
    marker_array.markers.push_back(marker);

    // Text Marker
    marker_text.id = seen_landmarks_[j];
    marker_text.text = marker_text.id;
    marker_array_text.markers.push_back(marker_text);
  }

  old_fiducials_observed_marker_publisher_->publish(marker_array);
  old_fiducials_observed_marker_publisher_->publish(marker_array_text);
}

// ----------------------------------------------------------------------------
void G2OBasedMapping::updateLocalization()
{
  TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = get_clock()->now();
  transform.child_frame_id = "base_link_g2o";
  transform.transform.translation.x = x_(0);
  transform.transform.translation.y = x_(1);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, x_(2));

  transform.transform.rotation = tf2::toMsg(q);

  transform_broadcaster_->sendTransform(transform);
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::publishMap()
{
  data_mutex_.lock();
  
  graph_map_.header.stamp = get_clock()->now();
  graph_map_.info.map_load_time = get_clock()->now();

  int map_size = graph_map_.info.width * graph_map_.info.height;
  graph_map_.data = std::vector<int8_t>(map_size, 0);

  for(size_t j = 0; j < robot_pose_ids_.size(); j++)
  {
    std::vector<double> data;
    graph_.vertex(robot_pose_ids_[j])->getEstimateData(data);

    g2o::OptimizableGraph::Data* d
      = graph_.vertex(robot_pose_ids_[j])->userData();

    g2o::RawLaser* rawLaser = dynamic_cast<g2o::RawLaser*>(d);

    if (rawLaser)
    {
      float angle = rawLaser->laserParams().firstBeamAngle;

      for(
        std::vector<double>::const_iterator i = rawLaser->ranges().begin();
        i != rawLaser->ranges().end();
        i++
      )
      {
        Point32 p;
        float x = *i * cos(angle);
        float y = *i * sin(angle);

        p.x = data[0] + x * cos(data[2]) - y * sin(data[2])
          - graph_map_.info.origin.position.x;

        p.y = data[1] + x * sin(data[2]) + y * cos(data[2])
          - graph_map_.info.origin.position.y;

        angle += rawLaser->laserParams().angularStep;

        unsigned int map_x = p.x / graph_map_.info.resolution;
        unsigned int map_y = p.y / graph_map_.info.resolution;

        if (
          map_x < graph_map_.info.width &&
          map_y < graph_map_.info.height
        )
        {
          graph_map_.data[map_y * graph_map_.info.width + map_x] = (int8_t) 100;
        }
      }
    }
  }

  map_publisher_->publish(graph_map_);

  data_mutex_.unlock();
}

// -----------------------------------------------------------------------------
double G2OBasedMapping::yawFromQuaternion(const Quaternion& quaternion)
{
  double roll;
  double pitch;
  double yaw;

  tf2::Quaternion q;
  tf2::fromMsg(quaternion, q);

  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::odomCallback(const Odometry::ConstSharedPtr& msg)
{
  data_mutex_.lock();
  updateOdometry(msg);
  data_mutex_.unlock();
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::laserCallback(const LaserScan::ConstSharedPtr& msg)
{
  data_mutex_.lock();
  updateLaser(msg);
  data_mutex_.unlock();
}

// -----------------------------------------------------------------------------
void G2OBasedMapping::initialPoseCallback(
  const PoseWithCovarianceStamped::ConstSharedPtr& msg
)
{
  data_mutex_.lock();

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double yaw = yawFromQuaternion(msg->pose.pose.orientation);

  RCLCPP_INFO_STREAM(
    get_logger(),
    "initialPoseCallback x: " << x << ", y: " << y << ", theta: " << yaw
  ); 

  init(x, y, yaw);

  data_mutex_.unlock();
}

} /* namespace tug_g2o_based_mapping */