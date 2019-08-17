#ifndef PYRAMID_CENTRAL_COMMON_H
#define PYRAMID_CENTRAL_COMMON_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <pyramid_msgs/common.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "pyramid_central/parameters.h"

namespace system_commander
{

struct EigenMultiDOFJointTrajectory
{
    EigenMultiDOFJointTrajectory()
        :timestamp_ns(-1),
         time_from_start_ns(0),
         position_ET(Eigen::Vector3d::Zero()),
         velocity_ET(Eigen::Vector3d::Zero()),
         acceleration_ET(Eigen::Vector3d::Zero()),
         orientation_ET(Eigen::Quaterniond::Identity()),
         angular_velocity_ET(Eigen::Vector3d::Zero()),
         angular_acceleration_ET(Eigen::Vector3d::Zero()){ }

    EigenMultiDOFJointTrajectory(int64_t _time_from_start_ns,
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
/*
inline void eigenMultiDOFJointTrajectoryFromMsg(const trajectory_msgs::MultiDOFJointTrajectoryPtr& msg,
                                               EigenMultiDOFJointTrajectory* multidoftrajectory)
{
    multidoftrajectory->timestamp_ns
        = msg->header.stamp.toNSec();
    multidoftrajectory->position_ET
        = pyramid_msgs::vector3FromMsg(msg->points.transforms.translation);
    multidoftrajectory->velocity_ET
        = pyramid_msgs::vector3FromMsg(msg->points.velocities[0].linear);
    multidoftrajectory->acceleration_ET
        = pyramid_msgs::vector3FromMsg(msg->points.accelerations[0].linear);
    multidoftrajectory->acceleration_ET
        = pyramid_msgs::quaternionFromMsg(msg->points.transforms[0].rotation);
    multidoftrajectory->angular_velocity_ET
        = pyramid_msgs::vector3FromMsg(msg->points.velocities[0].angular);
    multidoftrajectory->angular_acceleration_ET
        = pyramid_msgs::vector3FromMsg(msg->points.accelerations[0].angular);
}
*/
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

inline void getEulerAnglesFromQuaternion(const Eigen::Quaternion<double>& q,
                                         Eigen::Vector3d* euler_angles) {
  {
    assert(euler_angles != NULL);

    *euler_angles << atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
        asin(2.0 * (q.w() * q.y() - q.z() * q.x())),
        atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
              1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }
}

inline void skewMatrixFromVector(Eigen::Vector3d& vector, Eigen::Matrix3d* skew_matrix) {
  *skew_matrix << 0, -vector.z(), vector.y(),
                  vector.z(), 0, -vector.x(),
                  -vector.y(), vector.x(), 0;
}

inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                                 EigenOdometry* odometry) {
  odometry->position_EO = pyramid_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation_EO = pyramid_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity_EO = pyramid_msgs::vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity_EO = pyramid_msgs::vector3FromMsg(msg->twist.twist.angular);
}

inline void CalculateRotationMatrix(const Eigen::Quaterniond& orientation,
                                          Eigen::Matrix3d& rotation_matrix)
{
    rotation_matrix = orientation.toRotationMatrix();
}

inline void CalculateGlobalInertia(const Eigen::Matrix3d* rotation_matrix,
                                   const Eigen::Matrix3d* global_inertia)
{
    global_inertia = rotation_matrix * vehicle_parameters_.inertia_ * rotation_matrix.transpose();
}

inline void CalculateAngularMappingMatrix(const Eigen::Quaterniond& orientation)
{
    Eigen::Vector3d euler_angle;
    getEulerAnglesFromQuaternion(orientation, euler_angle);

    angular_mapping_matrix << 1,  0,                   -sin(euler_angle(1)),
                              0,  cos(euler_angle(0)),  cos(euler_angle(1))*sin(euler_angle(0)),
                              0, -sin(euler_angle(0)),  cos(euler_angle(1))*cos(euler_angle(0));
}

inline void CalculateDrivativeAngularMappingMatrix(const Eigen::Quaterniond& orientation)
{
    Eigen::Vector3d euler_angle;
    getEulerAnglesFromQuaternion(orientation, euler_angle);

    angular_mapping_matrix << 1,  0,                   -sin(euler_angle(1)),
                              0,  cos(euler_angle(0)),  cos(euler_angle(1))*sin(euler_angle(0)),
                              0, -sin(euler_angle(0)),  cos(euler_angle(1))*cos(euler_angle(0));
}

inline void CalculateSpatialInertiaMatrix(const VehicleParameters& vehicle_parameters,
                                          const Eigen::Matrix3d* global_inertia,
                                          const Eigen::Matrix3d* angular_mapping_matrix,
                                          const Eigen::MatrixXd* spatial_mass_matrix)
{
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    spatial_mass_matrix.block<3, 3>(0, 0)
        = I.array() * vehicle_parameters.mass_ ;
    spatial_mass_matrix.block<3, 3>(3, 3)
        = angular_mapping_matrix.transpose() * global_inertia * angular_mapping_matrix;
}

inline void CalculateCentrifugalCoriolisMatrix(const Eigen::Vector3d* angular_velocity,
                                               const Eigen::Matrix3d* global_inertia,
                                               const Eigen::Matrix3d* angular_mapping_matrix,
                                               const Eigen::MatrixXd* centrifugal_coriolis_matrix)
{
    Eigen::Matrix3d skew_matrix;
    skewMatrixFromVector(angular_velocity, skew_matrix);
    Eigen::Matrix3d derivative_angular_mapping_matrix;
    CalculateDrivativeAngularMappingMatrix(derivative_angular_mapping_matrix);
    Eigen::Matrix3d C;
    C = angular_mapping_matrix.transpose()*global_inertia*derivative_angular_mapping_matrix
        +angular_mapping_matrix.transpose()*skew_matrix*global_inertia*angular_mapping_matrix;

    centrifugal_coriolis_matrix.block<3, 3>(3, 3) = C;
}

inline void CalculateJacobian(const TetherConfiguration& tether_states,
                              const RotorConfiguration& rotor_configuration,
                              const Eigen::Matrix3d* rotation_matrix
                              const Eigen::MatrixXd* jacobian)
{
    Eigen::VectorXd J;
    J.resize(6);
    jacobian->resize(4, 6);
    unsigned int i = 0;
    for (const Tether& tether : tether_states.tethers)
    {

        J.block<3, 1>(0, 0) = tether.direction_;
        J.block<3, 1>(3, 0) = rotation_matrix * tether.mounting_pos_;
        jacobian.block<1, 3>(i, 0) = J;

        ++i;
    }
}

inline void CalculateWrench(const RotorConfiguration& rotor_configuration,
                            const EigenOdometry& odometry,
                            const Eigen::MatrixXd* spatial_mass_matrix,
                            const Eigen::MatrixXd* centrifugal_coriolis_matrix,
                            const Eigen::VectorXd* input_acc,
                            const Eigen::VectorXd* wrench)
{
    Eigen::VectorXd V;
    V.block<3, 1>(0, 0) = odometry.velocity_EO;
    V.block<3, 1>(3, 0) = odometry.angular_velocity_EO;
    Eigen::VectorXd G = Eigen::VectorXd::Zero(6);
    G(2) = rotor_configuration.mass_ * kDefaultGravity;
    wrench = spatial_mass_matrix * input_acc
             + centrifugal_coriolis_matrix * V + G;
}

} //namespace system_commander

#endif //PYRAMID_CENTRAL_COMMON_H
