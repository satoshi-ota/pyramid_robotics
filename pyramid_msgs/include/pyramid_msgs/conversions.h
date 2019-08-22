#ifndef PYRAMID_MSGS_CONVERSIONS_H
#define PYRAMID_MSGS_CONVERSIONS_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/WrenchStamped.h>

#include "pyramid_msgs/common.h"
#include "pyramid_msgs/pyramid_eigen_msgs.h"

namespace pyramid_msgs
{

inline void eigenThrustFromMsg(const geometry_msgs::WrenchStampedPtr& msg,
                                         pyramid_msgs::EigenWrenchStamped* thrust)
{
    assert(thrust != NULL);
    thrust->timestamp_ns = msg->header.stamp.toNSec();
    thrust->force_ET = vector3FromMsg(msg->wrench.force);
    thrust->torque_ET = vector3FromMsg(msg->wrench.torque);
}
/*
inline void msgMultiDofJointTrajectoryPointFromEigen(
    const EigenTrajectoryPoint& trajectory_point,
    trajectory_msgs::MultiDOFJointTrajectoryPoint* msg) {
  assert(msg != NULL);

  msg->time_from_start.fromNSec(trajectory_point.time_from_start_ns);
  msg->transforms.resize(1);
  msg->velocities.resize(1);
  msg->accelerations.resize(1);

  vectorEigenToMsg(trajectory_point.position_W,
                   &msg->transforms[0].translation);
  quaternionEigenToMsg(trajectory_point.orientation_W_B,
                       &msg->transforms[0].rotation);
  vectorEigenToMsg(trajectory_point.velocity_W, &msg->velocities[0].linear);
  vectorEigenToMsg(trajectory_point.angular_velocity_W,
                   &msg->velocities[0].angular);
  vectorEigenToMsg(trajectory_point.acceleration_W,
                   &msg->accelerations[0].linear);
}

inline void msgMultiDofJointTrajectoryFromEigen(
    const EigenTrajectoryPoint& trajectory_point, const std::string& link_name,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  assert(msg != NULL);
  trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
  msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &point_msg);

  msg->joint_names.clear();
  msg->points.clear();
  msg->joint_names.push_back(link_name);
  msg->points.push_back(point_msg);
}

inline void msgMultiDofJointTrajectoryFromEigen(
    const EigenTrajectoryPoint& trajectory_point,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  msgMultiDofJointTrajectoryFromEigen(trajectory_point, "base_link", msg);
}

// Convenience method to quickly create a trajectory from a single waypoint.
inline void msgMultiDofJointTrajectoryFromPositionYaw(
    const Eigen::Vector3d& position, double yaw,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  assert(msg != NULL);

  EigenTrajectoryPoint point;
  point.position_W = position;
  point.setFromYaw(yaw);

  msgMultiDofJointTrajectoryFromEigen(point, msg);
}
*/
} //namespace pyramid_msgs

#endif //PYRAMID_MSGS_CONVERSIONS_H
