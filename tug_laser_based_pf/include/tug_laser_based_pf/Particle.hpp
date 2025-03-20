#ifndef _TUG_LASER_BASED_PF__PARTICLE_HPP_
#define _TUG_LASER_BASED_PF__PARTICLE_HPP_

#include "geometry_msgs/msg/pose.hpp"

namespace tug_laser_based_pf
{

class Particle
{
  // Directives ----------------------------------------------------------------
  private:
    using Pose = geometry_msgs::msg::Pose;

  // Variables -----------------------------------------------------------------
  private:
    Pose pose_;
    double weight_;

  // Methods -------------------------------------------------------------------
  public:
    Particle();
    ~Particle();

    inline Pose getPose() const { return pose_; }
    inline double getWeight() const { return weight_; }
    inline double getX() const { return pose_.position.x; }
    inline double getY() const { return pose_.position.y; }
    double getTheta() const;

    void updatePose(double x, double y, double theta);
    inline void updateWeight(double weight) { weight_ = weight; }
};

} /* namespace tug_laser_based_pf */

#endif /* _TUG_LASER_BASED_PF__PARTICLE_HPP_ */