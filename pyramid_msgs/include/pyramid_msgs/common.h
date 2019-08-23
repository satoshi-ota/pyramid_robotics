#ifndef PYRAMID_MSGS_COMMON_H
#define PYRAMID_MSGS_COMMON_H

#include <Eigen/Eigen>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

namespace pyramid_msgs
{

inline void quaternionEigenToMsg(const Eigen::Quaterniond& eigen,
                                       geometry_msgs::Quaternion* msg)
{
    assert(msg != NULL);
    msg->x = eigen.x();
    msg->y = eigen.y();
    msg->z = eigen.z();
    msg->w = eigen.w();
}

inline void getEulerAnglesFromQuaternion(const Eigen::Quaternion<double>& q,
                                               Eigen::Vector3d* euler_angles)
{
    assert(euler_angles != NULL);
    *euler_angles << atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
                           asin(2.0 * (q.w() * q.y() - q.z() * q.x())),
                           atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                           1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline double yawFromQuaternion(const Eigen::Quaterniond& q)
{
    return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline Eigen::Vector3d vector3FromMsg(const geometry_msgs::Vector3& msg)
{
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Vector3d vector3FromPointMsg(const geometry_msgs::Point& msg)
{
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Quaterniond quaternionFromYaw(double yaw) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
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

inline void vectorEigenToMsg(const Eigen::Vector3d& eigen,
                             geometry_msgs::Vector3* msg)
{
  assert(msg != NULL);
  msg->x = eigen.x();
  msg->y = eigen.y();
  msg->z = eigen.z();
}

} //namespace pyramid_msgs

#endif //PYRAMID_MSGS_COMMON_H
