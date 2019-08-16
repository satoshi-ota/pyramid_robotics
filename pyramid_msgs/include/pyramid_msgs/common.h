#ifndef PYRAMID_MSGS_COMMON_H
#define PYRAMID_MSGS_COMMON_H

#include <Eigen/Eigen>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include "pyramid_msgs/conversions.h"

namespace pyramid_msgs
{

inline Eigen::Vector3d vector3FromMsg(const geometry_msgs::Vector3& msg)
{
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Quaterniond quaternionFromMsg(const geometry_msgs::Quaternion& msg)
{
  Eigen::Quaterniond quaternion(msg.w, msg.x, msg.y, msg.z);
  if (quaternion.norm() < std::numeric_limits<double>::epsilon())
  {
    quaternion.setIdentity();
  }
  else
  {
    quaternion.normalize();
  }
  return quaternion;
}

} //namespace pyramid_msgs

#endif //PYRAMID_MSGS_COMMON_H
