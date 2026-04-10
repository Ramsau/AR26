#include "tug_laser_based_pf/Particle.hpp"

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tug_laser_based_pf
{

// -----------------------------------------------------------------------------
Particle::Particle()
  : weight_(0.0)
{

}

// -----------------------------------------------------------------------------
Particle::~Particle()
{

}

// -----------------------------------------------------------------------------
double Particle::getTheta() const
{
  tf2::Quaternion q;
  tf2::fromMsg(pose_.orientation, q);

  tf2::Matrix3x3 m(q);
  double roll;
  double pitch;
  double yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

// -----------------------------------------------------------------------------
void Particle::updatePose(double x, double y, double theta)
{
  pose_.position.x = x;
  pose_.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);

  pose_.orientation = tf2::toMsg(q);
}

} /* namespace tug_laser_based_pf */