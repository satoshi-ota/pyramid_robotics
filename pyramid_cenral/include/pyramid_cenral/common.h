#ifndef PYRAMID_CENTRAL_COMMON_H
#define PYRAMID_CENTRAL_COMMON_H

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <trajectry_msgs/MultiDOFJointTrajectry.h>

namespace system_commander
{

struct EigenMultiDOFJointTrajectry
{
    EigenMultiDOFJointTrajectry()
        :timestamp_ns(-1),
         time_from_start_ns(0),
         position_ET(Eigen::Vector3d::Zero()),
         velocity_ET(Eigen::Vector3d::Zero()),
         acceleration_ET(Eigen::Vector3d::Zero()),
         orientation_ET(Eigen::Quaterniond::Identity()),
         angular_velocity_ET(Eigen::Vector3d::Zero()),
         angular_acceleration_ET(Eigen::Vector3d::Zero()){ }

    EigenMultiDOFJointTrajectry(int64_t _time_from_start_ns,
                                const Eigen::Vector3d& _position,
                                const Eigen::Vector3d& _velocity,
                                const Eigen::Vector3d& _acceleration,
                                const Eigen::Quaterniond& _orientation,
                                const Eigen::Vector3d& _angular_velocity,
                                const Eigen::Vector3d& _angular_acceleration)
        :time_from_start_ns(_time_from_start_ns),
         position_ET(_position),
         velocity_ET(_velocity),
         acceleration_ET(_acceleration),
         orientation_ET(_orientation),
         angular_velocity_ET(_angular_velocity),
         angular_acceleration_ET(_angular_acceleration){ }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int64_t timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
    int64_t time_from_start_ns;
    Eigen::Vector3d position_ET;
    Eigen::Vector3d velocity_ET;
    Eigen::Vector3d acceleration_ET;
    Eigen::Quaterniond orientation_ET;
    Eigen::Vector3d angular_velocity_ET;
    Eigen::Vector3d angular_acceleration_ET;
};

inline void eigenMultiDOFJointTrajectryFromMsg(const trajectry_msgs::MultiDOFJointTrajectryPtr& msg,
                                               EigenMultiDOFJointTrajectry* multidoftrajectry)
{
    multidoftrajectry->timestamp_ns
        = msg->header.stamp.toNSec();
    multidoftrajectry->position_ET
        = pyramid_msgs::vector3FromMsg(msg->transforms.translation);
    multidoftrajectry->velocity_ET
        = pyramid_msgs::vector3FromMsg(msg->velocities.linear);
    multidoftrajectry->acceleration_ET
        = pyramid_msgs::vector3FromMsg(msg->accelerations.linear);
    multidoftrajectry->acceleration_ET
        = pyramid_msgs::QuaternionFromMsg(msg->transforms.rotation);
    multidoftrajectry->angular_velocity_ET
        = pyramid_msgs::vector3FromMsg(msg->velocities.angular);
    multidoftrajectry->angular_acceleration_ET
        = pyramid_msgs::vector3FromMsg(msg->accelerations.angular);
}

struct EigenOdometry {
    EigenOdometry()
        :timestamp_ns(-1),
         position_EO(Eigen::Vector3d::Zero()),
         orientation_EO(Eigen::Quaterniond::Identity()),
         velocity_EO(Eigen::Vector3d::Zero()),
         angular_velocity_EO(Eigen::Vector3d::Zero()) {}

    EigenOdometry(const Eigen::Vector3d& _position,
                  const Eigen::Quaterniond& _orientation,
                  const Eigen::Vector3d& _velocity_body,
                  const Eigen::Vector3d& _angular_velocity)
        :position_EO(_position),
         orientation_EO(_orientation),
         velocity_EO(_velocity_body),
         angular_velocity_EO(_angular_velocity) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int64_t timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
    Eigen::Vector3d position_EO;
    Eigen::Quaterniond orientation_EO;
    Eigen::Vector3d velocity_EO;
    Eigen::Vector3d angular_velocity_EO;
    Eigen::Matrix<double, 6, 6> pose_covariance_;
    Eigen::Matrix<double, 6, 6> twist_covariance_;
};

inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                                 EigenOdometry* odometry) {
  odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
}

inline void CalculateRotationMatrix(const Eigen::Quaterniond& orientation,
                                          Eigen::Matrix3d& rotation_matrix)
{
    rotation_matrix = orientation.toRotationMatrix();
}

inline void CalculateSpatialInertiaMatrix(const RotorConfiguration& rotor_configuration)
{

}

inline void CalculateCentrifugalCoriolisMatrix()
{

}

inline void CalculateJacobian(const TetherConfiguration& tether_configuration,
                              const RotorConfiguration& rotor_configuration)
{

}

} //namespace system_commander

#endif //PYRAMID_CENTRAL_COMMON_H
