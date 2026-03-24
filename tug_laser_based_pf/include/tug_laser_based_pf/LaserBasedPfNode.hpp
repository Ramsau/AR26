#ifndef _TUG_LASER_BASED_PF__LASER_BASED_PF_NODE_HPP_
#define _TUG_LASER_BASED_PF__LASER_BASED_PF_NODE_HPP_

#include <memory>
#include <mutex>
#include <vector>

#include "Eigen/Dense"

#include "rclcpp/node.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/srv/get_map.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "visualization_msgs/msg/marker.hpp"

#include "tug_laser_based_pf/Particle.hpp"

namespace tug_laser_based_pf
{

class LaserBasedPfNode : public rclcpp::Node
{
  // Directives ----------------------------------------------------------------
  private:
    using Pose = geometry_msgs::msg::Pose;
    using PoseArray = geometry_msgs::msg::PoseArray;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using PoseWithCovarianceStamped
      = geometry_msgs::msg::PoseWithCovarianceStamped;
    using TransformStamped = geometry_msgs::msg::TransformStamped;

    using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
    using Odometry = nav_msgs::msg::Odometry;
    using GetMap = nav_msgs::srv::GetMap;

    using LaserScan = sensor_msgs::msg::LaserScan;

    using Marker = visualization_msgs::msg::Marker;

  // Variables -----------------------------------------------------------------
  private:
    std::mutex data_lock_;

    tf2_ros::Buffer::SharedPtr tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pose_broadcaster_;

    OccupancyGrid occupancy_grid_;
    Odometry last_odometry_;
    
    //LaserScan laser_info_;

    Eigen::MatrixXd x_;

    int num_partivles_;
    std::vector<Particle> particles_;
    int max_x_position_;
    int max_y_position_;

    // Publisher
    rclcpp::Publisher<PoseArray>::SharedPtr particles_pub_;
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_with_cov_pub_;
    rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<Marker>::SharedPtr vis_pub_;

    // Subscriber
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;

    // Service Client
    rclcpp::Client<GetMap>::SharedPtr map_client_;

  // Methods -------------------------------------------------------------------
  public:
    LaserBasedPfNode();
    ~LaserBasedPfNode();

    void updateOdometry(const Odometry& odom);
    void updateLaser(const LaserScan& scan);
    void resetLocalization(double x, double y, double theta);

    void updateLocalization(
      Eigen::MatrixXd& x,
      const std::vector<Particle>& particles
    );

    void laserCallback(const LaserScan::ConstSharedPtr& msg);
    void odometryCallback(const Odometry::ConstSharedPtr& msg);
    void initialPoseCallback(
      const PoseWithCovarianceStamped::ConstSharedPtr& msg
    );
    void mapCallback();
  
  private:
    void publishParticles(const std::vector<Particle>& particles);
    std::vector<Pose> getParticlePositions(
      const std::vector<Particle>& particles
    );
    void publishPose(
      Eigen::MatrixXd& x,
      const std::vector<Particle>& particles
    );
    void generateUncertaintyMarker(
      Marker& marker,
      const Eigen::Matrix2f& uncertainly_mat,
      const Eigen::Vector2f& position
    );

    void initParticles(const OccupancyGrid& map);
    void normalizeParticleWeights();
    void resamplingParticles();
    double probNormalDistribution(double a, double variance);
    double sampleNormalDistribution(double variance);

  private:
    double randomDouble();
    double normalizeAngle(double angle);
    double alpha_1 = 0.001;
    double alpha_2 = 0.001;
    double alpha_3 = 0.001;
    double alpha_4 = 0.001;
    double sigma_hit = 2.0;
    std::vector<double> likelihood_field_;
    nav_msgs::msg::OccupancyGrid likelihood_field_map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr likelihood_field_pub_;
    struct Point {
      int x;
      int y;
    };
};

} /* namespace tug_laser_based_pf */

#endif /* _TUG_LASER_BASED_PF__LASER_BASED_PF_NODE_HPP_ */