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
#include <geometry_msgs/WrenchStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include "pyramid_central/parameters.h"

namespace system_commander
{

struct EigenThrust
{
    EigenThrust()
    :timestamp_ns(-1),
     force(Eigen::Vector3d::Zero()),
     torque(Eigen::Vector3d::Zero()){ }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int64_t timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
    Eigen::Vector3d force;
    Eigen::Vector3d torque;
};

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

inline void eigenMultiDOFJointTrajectoryFromMsg(
                                    const trajectory_msgs::MultiDOFJointTrajectoryPtr& msg,
                                          EigenMultiDOFJointTrajectory* multidoftrajectory)
{
    if (msg->points[0].transforms.empty())
    {
        ROS_ERROR("MultiDofJointTrajectoryPoint is empty.");
        return;
    }

    if (msg->points[0].transforms.size() > 1)
    {
        ROS_WARN(
            "MultiDofJointTrajectoryPoint message should have one joint, but has "
            "%lu. Using first joint.",
            msg->points[0].transforms.size());
    }

    multidoftrajectory->timestamp_ns
        = msg->header.stamp.toNSec();

    multidoftrajectory->position_ET
        = pyramid_msgs::vector3FromMsg(msg->points[0].transforms[0].translation);
    multidoftrajectory->velocity_ET
        = pyramid_msgs::vector3FromMsg(msg->points[0].velocities[0].linear);
    multidoftrajectory->acceleration_ET
        = pyramid_msgs::vector3FromMsg(msg->points[0].accelerations[0].linear);
    multidoftrajectory->orientation_ET
        = pyramid_msgs::quaternionFromMsg(msg->points[0].transforms[0].rotation);
    multidoftrajectory->angular_velocity_ET
        = pyramid_msgs::vector3FromMsg(msg->points[0].velocities[0].angular);
    multidoftrajectory->angular_acceleration_ET
        = pyramid_msgs::vector3FromMsg(msg->points[0].accelerations[0].angular);
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

inline void getEulerAnglesFromQuaternion(const Eigen::Quaternion<double>& q,
                                               Eigen::Vector3d* euler_angles)
{
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
                                       EigenOdometry* odometry)
{
  odometry->position_EO = pyramid_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation_EO = pyramid_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity_EO = pyramid_msgs::vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity_EO = pyramid_msgs::vector3FromMsg(msg->twist.twist.angular);
}

inline void eigenThrustToMsg(const EigenThrust& thrust,
                                   geometry_msgs::WrenchStamped& thrust_msg)
{
    pyramid_msgs::vectorEigenToMsg(thrust.force, &thrust_msg.wrench.force);
    pyramid_msgs::vectorEigenToMsg(thrust.torque, &thrust_msg.wrench.torque);
}

inline void CalculateRotationMatrix(const Eigen::Quaterniond& orientation,
                                          Eigen::Matrix3d* rotation_matrix)
{
    *rotation_matrix = orientation.toRotationMatrix();
}

inline void CalculateGlobalInertia(const Eigen::Matrix3d& inertia,
                                   const Eigen::Matrix3d& rotation_matrix,
                                         Eigen::Matrix3d* global_inertia)
{
    *global_inertia = rotation_matrix * inertia * rotation_matrix.transpose();
}

inline void CalculateAngularMappingMatrix(const Eigen::Quaterniond& orientation,
                                                Eigen::Matrix3d* angular_mapping_matrix)
{
    Eigen::Vector3d angles;
    getEulerAnglesFromQuaternion(orientation, &angles);

    *angular_mapping_matrix << 1,  0,              -sin(angles(1)),
                               0,  cos(angles(0)),  cos(angles(1))*sin(angles(0)),
                               0, -sin(angles(0)),  cos(angles(1))*cos(angles(0));
}

inline void CalculateDrivativeAngularMappingMatrix(
                                            const Eigen::Quaterniond& orientation,
                                                  Eigen::Matrix3d* derivative_angular_mapping_matrix)
{
    Eigen::Vector3d angles;
    getEulerAnglesFromQuaternion(orientation, &angles);

    *derivative_angular_mapping_matrix
        << 1,  0,              -cos(angles(1)),
           0, -sin(angles(0)), -sin(angles(1))*sin(angles(0))+cos(angles(1))*sin(angles(0)),
           0, -cos(angles(0)), -sin(angles(1))*cos(angles(0))-cos(angles(1))*sin(angles(0));
}

inline void CalculateSpatialInertiaMatrix(const SystemParameters& system_parameters,
                                          const Eigen::Matrix3d& global_inertia,
                                          const Eigen::Matrix3d& angular_mapping_matrix,
                                                Eigen::MatrixXd* spatial_mass_matrix)
{
    Eigen::Matrix<double, 6, 6> S_I_M = Eigen::MatrixXd::Zero(6, 6);

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    S_I_M.block<3, 3>(0, 0)
        = system_parameters.mass_ * I.array();
    S_I_M.block<3, 3>(3, 3)
        = angular_mapping_matrix.transpose() * global_inertia * angular_mapping_matrix;

    *spatial_mass_matrix = S_I_M;

}

inline void CalculateCentrifugalCoriolisMatrix(const Eigen::Vector3d& angular_velocity,
                                               const Eigen::Matrix3d& global_inertia,
                                               const Eigen::Matrix3d& angular_mapping_matrix,
                                               const Eigen::Matrix3d&
                                                     derivative_angular_mapping_matrix,
                                                     Eigen::MatrixXd* centrifugal_coriolis_matrix)
{
    Eigen::Matrix<double, 6, 6> C_C_M = Eigen::MatrixXd::Zero(6, 6);

    Eigen::Matrix3d skew_matrix;
    Eigen::Vector3d omega = angular_velocity;
    skewMatrixFromVector(omega, &skew_matrix);
    Eigen::Matrix3d C;
    C = angular_mapping_matrix.transpose() * global_inertia * derivative_angular_mapping_matrix
        + angular_mapping_matrix.transpose() * skew_matrix * global_inertia * angular_mapping_matrix;
    C_C_M.block<3, 3>(3, 3) = C;

    *centrifugal_coriolis_matrix = C_C_M;
}

inline void CalculateJacobian(const TetherConfiguration& tether_configuration,
                              const Eigen::Matrix3d& rotation_matrix,
                                    Eigen::MatrixXd* jacobian)
{
    jacobian->resize(4, 6);
    jacobian->setZero();

    Eigen::Matrix<double, 6, 4> J = Eigen::MatrixXd::Zero(6, 4);

    unsigned int i = 0;
    for (const Tether& tether : tether_configuration.tethers)
    {
        J.block<3, 1>(0, i) = -tether.direction;
        J.block<3, 1>(3, i) =  tether.direction.cross(rotation_matrix*tether.mounting_pos);

        ++i;
    }
    *jacobian = J.transpose();
}

inline void CalculateJacobianTilde(const Eigen::Matrix3d& angular_mapping_matrix,
                                   const Eigen::MatrixXd& jacobian,
                                         Eigen::MatrixXd* jacobian_tilde)
{
    jacobian_tilde->resize(4, 6);
    jacobian_tilde->setZero();

    Eigen::Matrix<double, 6, 6> S1 = Eigen::MatrixXd::Zero(6, 6);

    S1.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    S1.block<3, 3>(3, 3) = angular_mapping_matrix;

    *jacobian_tilde = jacobian * S1;
}

inline void CalculateWrench(const SystemParameters& system_parameters,
                            const EigenOdometry& odometry,
                            const Eigen::MatrixXd& spatial_mass_matrix,
                            const Eigen::MatrixXd& centrifugal_coriolis_matrix,
                            const Eigen::VectorXd& input_acceleration,
                                  Eigen::VectorXd* wrench)
{
    Eigen::VectorXd V = Eigen::VectorXd::Zero(6);
    V.block<3, 1>(0, 0) = odometry.velocity_EO;
    V.block<3, 1>(3, 0) = odometry.angular_velocity_EO;

    Eigen::VectorXd G = Eigen::VectorXd::Zero(6);
    G(2) = system_parameters.mass_ * kDefaultGravity;

    *wrench = spatial_mass_matrix * input_acceleration + centrifugal_coriolis_matrix * V + G;
}

inline void CalculatePosAttDelta(const EigenOdometry& odometry,
                                 const EigenMultiDOFJointTrajectory& desired_trajectory,
                                       Eigen::VectorXd* x_delta)
{
    Eigen::VectorXd pos_att_delta = Eigen::VectorXd::Zero(6);

    pos_att_delta.block<3, 1>(0, 0) = desired_trajectory.position_ET - odometry.position_EO;

    Eigen::Vector3d euler_angles;
    getEulerAnglesFromQuaternion(odometry.orientation_EO, &euler_angles);
    Eigen::Vector3d desired_euler_angles;
    getEulerAnglesFromQuaternion(desired_trajectory.orientation_ET, &desired_euler_angles);

    pos_att_delta.block<3, 1>(3, 0) = desired_euler_angles - euler_angles;

    *x_delta = pos_att_delta;
}

inline void CalculateVelocityDelta(const EigenOdometry& odometry,
                                   const EigenMultiDOFJointTrajectory& desired_trajectory,
                                         Eigen::VectorXd* v_delta)
{
    Eigen::VectorXd velocity_delta = Eigen::VectorXd::Zero(6);

    velocity_delta.block<3, 1>(0, 0)
        = desired_trajectory.velocity_ET - odometry.velocity_EO;
    velocity_delta.block<3, 1>(3, 0)
        = desired_trajectory.angular_velocity_ET - odometry.angular_velocity_EO;

    *v_delta = velocity_delta;
}

inline void EigenVectorToEigenThrust(const Eigen::VectorXd& controller_output,
                                           EigenThrust* thrust)
{
    thrust->force = controller_output.block<3, 1>(0, 0);
    thrust->torque = controller_output.block<3, 1>(3, 0);
}

} //namespace system_commander

#endif //PYRAMID_CENTRAL_COMMON_H
