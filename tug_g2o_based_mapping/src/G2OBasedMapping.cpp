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

namespace tug_g2o_based_mapping {
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
    laser_noise_(0, 0) = 0.8;
    laser_noise_(1, 1) = 0.8;
    laser_noise_(2, 2) = 5.0;

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

    // 1. Enter your odometry update here
    // 2. Keep track of the odometry updates to the robot position

    // global variable last_odometry_ contains the last odometry position estimation (ROS Odometry Messasge)
    // local variable oodom contains the current odometry position estimation (ROS Odometry Messasge)

    // global variable x_ holds your position (Eigen vector of size 3 [x,y,theta])
    double delta_x_odom = odom->pose.pose.position.x - last_odometry_.pose.pose.position.x;
    double delta_y_odom = odom->pose.pose.position.y - last_odometry_.pose.pose.position.y;
    double delta_theta = yawFromQuaternion(odom->pose.pose.orientation) - yawFromQuaternion(last_odometry_.pose.pose.orientation);

    double last_yaw = yawFromQuaternion(last_odometry_.pose.pose.orientation);
    double delta_x_robot = cos(last_yaw) * delta_x_odom + sin(last_yaw) * delta_y_odom;
    double delta_y_robot = - sin(last_yaw) * delta_x_odom + cos(last_yaw) * delta_y_odom;

    // take into account drifting orientation
    double delta_x_est = cos(x_(2)) * delta_x_robot - sin(x_(2)) * delta_y_robot;
    double delta_y_est = sin(x_(2)) * delta_x_robot + cos(x_(2)) * delta_y_robot;
    x_(0, 0) += delta_x_est;
    x_(1, 0) += delta_y_est;
    x_(2, 0) += delta_theta;

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

    // 1. Enter your laser scan update here

    auto last_vertex = dynamic_cast<g2o::VertexSE2*>(graph_.vertex(last_id_));
    if (!last_vertex) {
      RCLCPP_ERROR_STREAM(get_logger(), "last vertex is not a vertexSE2");
      return;
    }

    Eigen::Vector3<double> last_vertex_pose{
      last_vertex->estimate()[0],
      last_vertex->estimate()[1],
      last_vertex->estimate()[2]
    };
    Eigen::Vector2<double> dist = {
      x_(0) - last_vertex_pose(0),
      x_(1) - last_vertex_pose(1)
    };
    if (dist.norm() > laser_vertex_dist_ || abs(x_(2) - last_vertex_pose(2)) > laser_vertex_dtheta_) {
      // create new vertex and connect
      Eigen::Vector3<double> last_vertex_pose{
        last_vertex->estimate()[0],
        last_vertex->estimate()[1],
        last_vertex->estimate()[2]
      };
      double delta_x = x_(0) - last_vertex_pose(0);
      double delta_y = x_(1) - last_vertex_pose(1);
      double delta_theta = x_(2) - last_vertex_pose(2);

      g2o::RawLaser* last_laser = dynamic_cast<g2o::RawLaser*>(graph_.vertex(last_id_)->userData());
      std::shared_ptr<g2o::RawLaser> current_laser = std::make_shared<g2o::RawLaser>();
      current_laser->setLaserParams(*laser_params_);
      current_laser->setRanges(std::vector<double>(laser->ranges.begin(), laser->ranges.end()));

      double delta_x_robot = cos(last_vertex_pose(2)) * delta_x + sin(last_vertex_pose(2)) * delta_y;
      double delta_y_robot = - sin(last_vertex_pose(2)) * delta_x + cos(last_vertex_pose(2)) * delta_y;
      double delta_theta_robot = delta_theta;
      if (iterativeClosestPoint(*current_laser, *last_laser, delta_x_robot, delta_y_robot, delta_theta_robot)) {

        delta_x = cos(last_vertex_pose(2)) * delta_x_robot - sin(last_vertex_pose(2)) * delta_y_robot;
        delta_y = sin(last_vertex_pose(2)) * delta_x_robot + cos(last_vertex_pose(2)) * delta_y_robot;
        delta_theta = delta_theta_robot;
        addLaserVertex(
          last_vertex_pose(0) + delta_x,
          last_vertex_pose(1) + delta_y,
          last_vertex_pose(2) + delta_theta,
        *laser, last_id_ + 1, false);
        addLaserEdge(last_id_, last_id_ + 1,
          delta_x_robot,
          delta_y_robot,
        delta_theta_robot,
        laser_noise_);


        last_id_++;

        if (closeLoop(last_id_)) {
          optimizeGraph();
          setRobotToVertex(last_id_);
        }

      } else {
        RCLCPP_WARN_STREAM(get_logger(), "ICP failed");
      }
    }
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
  bool G2OBasedMapping::closeLoop(int id) {
    std::vector<double> vertex_pos;
    g2o::OptimizableGraph::Vertex * vertex = graph_.vertex(id);
    vertex->getEstimateData(vertex_pos);

    for (auto other_vertex_pair: graph_.vertices()) {
      int other_vertex_id = other_vertex_pair.first;
      // check identity
      if (other_vertex_id == id) {
        continue;
      }

      // check distance
      g2o::OptimizableGraph::Vertex * other_vertex = graph_.vertex(other_vertex_id);
      std::vector<double> other_vertex_pos;
      other_vertex->getEstimateData(other_vertex_pos);
      double dx = vertex_pos[0] - other_vertex_pos[0];
      double dy = vertex_pos[1] - other_vertex_pos[1];
      double distance = sqrt(dx * dx + dy * dy);
      if (distance > loop_closure_distance_threshold_) {
        continue;
      }

      // check neighbor
      for (auto edge: vertex->edges()) {
        if (edge->vertex(0)->id() == other_vertex_id || edge->vertex(1)->id() == other_vertex_id) {
          continue;
        }
      }

      g2o::RawLaser* vertex_laser = dynamic_cast<g2o::RawLaser*>(vertex->userData());
      g2o::RawLaser* other_vertex_laser = dynamic_cast<g2o::RawLaser*>(other_vertex->userData());
      double delta_x = vertex_pos[0] - other_vertex_pos[0];
      double delta_y = vertex_pos[1] - other_vertex_pos[1];
      double delta_theta = vertex_pos[2] - other_vertex_pos[2];


      double delta_x_robot = cos(other_vertex_pos[2]) * delta_x + sin(other_vertex_pos[2]) * delta_y;
      double delta_y_robot = - sin(other_vertex_pos[2]) * delta_x + cos(other_vertex_pos[2]) * delta_y;
      double delta_theta_robot = delta_theta;
      if (iterativeClosestPoint(*vertex_laser, *other_vertex_laser, delta_x_robot, delta_y_robot, delta_theta_robot)) {
        addLaserEdge(other_vertex_id, id, delta_x_robot, delta_y_robot, delta_theta_robot, laser_noise_);
        return true;
      }
    }
    return false;
  }


  // -----------------------------------------------------------------------------
  bool G2OBasedMapping::iterativeClosestPoint(const g2o::RawLaser &scan_p, const g2o::RawLaser &scan_q,
    double& delta_x, double& delta_y, double& delta_theta) {
    typedef Eigen::Vector2<double> Point;
    std::vector<Point> points_q;

    // precompute points of scan q
    for (int q_range_id = 0; q_range_id < scan_q.ranges().size(); q_range_id++) {
      if (std::isnan(scan_q.ranges()[q_range_id]) || std::isinf(scan_q.ranges()[q_range_id])) {
        continue;
      }
      if (q_range_id % icp_use_nth_point_ != 0) {
        continue;
      }
      double range = scan_q.ranges()[q_range_id];
      double angle = scan_q.laserParams().firstBeamAngle + scan_q.laserParams().angularStep * q_range_id;
      double x, y;
      laserScanToPoint(0, 0, 0, range, angle, x, y);
      points_q.push_back({x, y});
    }


    // iterate until convergence
    int iterations = 1;
    while (true) {
      std::map<int, int> closest_points{};
      double error = 0;

      // get closest point in q for each point in p
      std::vector<Point> points_p;
      Point centroid_p = {0, 0};
      Point centroid_q = {0, 0};
      int p_id = 0;
      for (int p_range_id = 0; p_range_id < scan_p.ranges().size(); p_range_id++) {
        if (std::isnan(scan_p.ranges()[p_range_id]) || std::isinf(scan_p.ranges()[p_range_id])) {
          continue;
        }
        if (p_range_id % icp_use_nth_point_ != 0) {
          continue;
        }
        double range = scan_p.ranges()[p_range_id];
        double angle = scan_p.laserParams().firstBeamAngle + scan_p.laserParams().angularStep * p_range_id;
        double x, y;
        laserScanToPoint(delta_x, delta_y, delta_theta, range, angle, x, y);
        int closest_point_id = -1;
        double closest_distance = std::numeric_limits<double>::max();
        for (int q_id = 0; q_id < points_q.size(); q_id++) {
          double distance = (x - points_q.at(q_id)(0)) * (x - points_q.at(q_id)(0)) + (y - points_q.at(q_id)(1)) * (y - points_q.at(q_id)(1));
          if (distance < closest_distance) {
            closest_distance = distance;
            closest_point_id = q_id;
          }
        }
        if (closest_distance < icp_max_distance_) {
          closest_points[p_id] = closest_point_id;
          error += closest_distance * closest_distance;

          centroid_p += Point(x, y);
          centroid_q += points_q.at(closest_point_id);

          points_p.push_back({x, y});
          p_id++;
        } else {
          RCLCPP_DEBUG_STREAM(get_logger(), "point " << p_range_id << " is too far away");
        }
      }
      centroid_p /= points_p.size();
      centroid_q /= points_q.size();
      error /= points_p.size();

      // calculate centered p and W
      Eigen::Matrix2<double> W = Eigen::Matrix2<double>::Zero();
      for (int p_id = 0; p_id < points_p.size(); p_id++) {
        Point p_centered = points_p.at(p_id) - centroid_p;
        Point q_centered = points_q.at(closest_points.at(p_id)) - centroid_q;
        W += p_centered * q_centered.transpose();
      }
      Eigen::Matrix2<double> U, V;
      Eigen::JacobiSVD<Eigen::Matrix2<double>> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
      U = svd.matrixU();
      V = svd.matrixV();
      Eigen::Matrix2<double> R = V * U.transpose();
      if (R.determinant() < 0) {
        V.col(1) *= -1;
        R = V * U.transpose();
      }
      // Eigen::Vector2<double> delta_p = R * (points_p.at(0) - points_q.at(closest_points.at(0)));
      RCLCPP_DEBUG_STREAM(get_logger(), "R:\n" << R);
      Eigen::Vector2<double> T = centroid_q - R * centroid_p;
      RCLCPP_DEBUG_STREAM(get_logger(), "T:\n" << T);
      RCLCPP_DEBUG_STREAM(get_logger(), "error: " << error);

      double dd_x = T(0);
      double dd_y = T(1);
      double dd_theta = atan2(R(1, 0), R(0, 0));

      delta_x += dd_x;
      delta_y += dd_y;
      delta_theta += dd_theta;
      if (sqrt(dd_x * dd_x + dd_y * dd_y) < icp_translation_convergence_threshold_ && dd_theta < icp_rotation_convergence_threshold_) {
        RCLCPP_DEBUG_STREAM(get_logger(), "final dx:" << delta_x << " dy:" << delta_y << " dtheta:" << delta_theta);
        return true;
      }
      RCLCPP_DEBUG_STREAM(get_logger(), "delta_x: " << delta_x << " delta_y: " << delta_y << " delta_theta: " << delta_theta);
      if (iterations++ > icp_max_iterations_) {
        return false;
      }
    }
  }

// -----------------------------------------------------------------------------
void G2OBasedMapping::laserScanToPoint(double robot_x, double robot_y, double robot_theta, double laser_range, double laser_angle, double& laser_x, double& laser_y) {
  laser_x = robot_x + laser_range * cos(laser_angle + robot_theta);
  laser_y = robot_y + laser_range * sin(laser_angle + robot_theta);
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