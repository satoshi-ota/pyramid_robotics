#ifndef PYRAMID_MSGS_CONVERSIONS_H
#define PYRAMID_MSGS_CONVERSIONS_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "pyramid_msgs/common.h"
#include "pyramid_msgs/pyramid_eigen_msgs.h"

namespace pyramid_msgs
{

inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                                       pyramid_msgs::EigenOdometry* odometry)
{
  odometry->position = pyramid_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation = pyramid_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity = pyramid_msgs::vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity = pyramid_msgs::vector3FromMsg(msg->twist.twist.angular);
}

inline void eigenThrustToMsg(const pyramid_msgs::EigenThrust& thrust,
                                   geometry_msgs::WrenchStamped& thrust_msg)
{
    pyramid_msgs::vectorEigenToMsg(thrust.force, &thrust_msg.wrench.force);
    pyramid_msgs::vectorEigenToMsg(thrust.torque, &thrust_msg.wrench.torque);
}

inline void eigenThrustFromMsg(const geometry_msgs::WrenchStampedPtr& msg,
                                     pyramid_msgs::EigenWrenchStamped* thrust)
{
    assert(thrust != NULL);
    thrust->timestamp_ns = msg->header.stamp.toNSec();
    thrust->force = vector3FromMsg(msg->wrench.force);
    thrust->torque = vector3FromMsg(msg->wrench.torque);
}

inline void vectorToEigenThrust(const Eigen::VectorXd& wrench,
                                      pyramid_msgs::EigenThrust* thrust)
{
    thrust->force = wrench.block<3, 1>(0, 0);
    thrust->torque = wrench.block<3, 1>(3, 0);
}

inline void eigenMultiDOFJointTrajectoryFromMsg(
                                const trajectory_msgs::MultiDOFJointTrajectoryPtr& msg,
                                      pyramid_msgs::EigenMultiDOFJointTrajectory*multidoftrajectory)
{
    multidoftrajectory->timestamp_ns
        = msg->header.stamp.toNSec();

    multidoftrajectory->position
        = pyramid_msgs::vector3FromMsg(msg->points[0].transforms[0].translation);
    multidoftrajectory->velocity
        = pyramid_msgs::vector3FromMsg(msg->points[0].velocities[0].linear);
    multidoftrajectory->acceleration
        = pyramid_msgs::vector3FromMsg(msg->points[0].accelerations[0].linear);
    multidoftrajectory->orientation
        = pyramid_msgs::quaternionFromMsg(msg->points[0].transforms[0].rotation);
    multidoftrajectory->angular_velocity
        = pyramid_msgs::vector3FromMsg(msg->points[0].velocities[0].angular);
    multidoftrajectory->angular_acceleration
        = pyramid_msgs::vector3FromMsg(msg->points[0].accelerations[0].angular);
}

inline void msgMultiDofJointTrajectoryPointFromEigen(const EigenTrajectoryPoint& trajectory_point,
                                                trajectory_msgs::MultiDOFJointTrajectoryPoint* msg)
{
    assert(msg != NULL);

    msg->time_from_start.fromNSec(trajectory_point.time_from_start_ns);
    msg->transforms.resize(1);
    msg->velocities.resize(1);
    msg->accelerations.resize(1);

    vectorEigenToMsg(trajectory_point.position, &msg->transforms[0].translation);
    quaternionEigenToMsg(trajectory_point.orientation, &msg->transforms[0].rotation);
    vectorEigenToMsg(trajectory_point.velocity, &msg->velocities[0].linear);
    vectorEigenToMsg(trajectory_point.angular_velocity, &msg->velocities[0].angular);
    vectorEigenToMsg(trajectory_point.acceleration, &msg->accelerations[0].linear);
}

inline void msgMultiDofJointTrajectoryFromEigen(const EigenTrajectoryPoint& trajectory_point,
                                                const std::string& link_name,
                                                      trajectory_msgs::MultiDOFJointTrajectory* msg)
{
    assert(msg != NULL);
    trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
    msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &point_msg);

    msg->joint_names.clear();
    msg->points.clear();
    msg->joint_names.push_back(link_name);
    msg->points.push_back(point_msg);
}

inline void msgMultiDofJointTrajectoryFromEigen(const EigenTrajectoryPoint& trajectory_point,
                                                      trajectory_msgs::MultiDOFJointTrajectory* msg)
{
  msgMultiDofJointTrajectoryFromEigen(trajectory_point, "base_link", msg);
}

inline void msgMultiDofJointTrajectoryFromPositionYaw(
    const Eigen::Vector3d& position, double yaw,
    trajectory_msgs::MultiDOFJointTrajectory* msg)
{
    assert(msg != NULL);

    EigenTrajectoryPoint point;
    point.position = position;
    point.setFromYaw(yaw);

    msgMultiDofJointTrajectoryFromEigen(point, msg);
}

} //namespace pyramid_msgs

#endif //PYRAMID_MSGS_CONVERSIONS_H
