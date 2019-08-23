#ifndef PYRAMID_CENTRAL_COMMON_H
#define PYRAMID_CENTRAL_COMMON_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <pyramid_msgs/common.h>
#include <pyramid_msgs/pyramid_eigen_msgs.h>
#include <geometry_msgs/WrenchStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "pyramid_central/parameters.h"

namespace system_commander
{

inline void skewMatrixFromVector(Eigen::Vector3d& vector, Eigen::Matrix3d* skew_matrix)
{
  *skew_matrix <<          0,-vector.z(),  vector.y(),
                  vector.z(),          0, -vector.x(),
                 -vector.y(), vector.x(),           0;
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
    pyramid_msgs::getEulerAnglesFromQuaternion(orientation, &angles);

    *angular_mapping_matrix << 1,  0,              -sin(angles(1)),
                               0,  cos(angles(0)),  cos(angles(1))*sin(angles(0)),
                               0, -sin(angles(0)),  cos(angles(1))*cos(angles(0));
}

inline void CalculateDrivativeAngularMappingMatrix(
                                            const Eigen::Quaterniond& orientation,
                                                  Eigen::Matrix3d* derivative_angular_mapping_matrix)
{
    Eigen::Vector3d angles;
    pyramid_msgs::getEulerAnglesFromQuaternion(orientation, &angles);

    *derivative_angular_mapping_matrix
        << 0,  0,              -cos(angles(1)),
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
        J.block<3, 1>(0, i) =  tether.direction;
        J.block<3, 1>(3, i) = -tether.direction.cross(rotation_matrix*tether.mounting_pos);

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
                            const pyramid_msgs::EigenOdometry& odometry,
                            const Eigen::MatrixXd& spatial_mass_matrix,
                            const Eigen::MatrixXd& centrifugal_coriolis_matrix,
                            const Eigen::VectorXd& input_acceleration,
                                  Eigen::VectorXd* wrench)
{
    Eigen::VectorXd G;
    double mg = system_parameters.mass_ * kDefaultGravity;
    G << 0.0, 0.0, mg, 0.0, 0.0, 0.0;

    *wrench = spatial_mass_matrix * input_acceleration
              + centrifugal_coriolis_matrix * odometry.getVel() + G;
}

inline void CalculatePosAttDelta(const pyramid_msgs::EigenOdometry& odometry,
                                 const pyramid_msgs::EigenMultiDOFJointTrajectory& desired_trajectory,
                                       Eigen::VectorXd* x_error)
{
    *x_error = desired_trajectory.getPosAtt() - odometry.getPosAtt();
}

inline void CalculateVelocityDelta(
                const pyramid_msgs::EigenOdometry& odometry,
                const pyramid_msgs::EigenMultiDOFJointTrajectory& desired_trajectory,
                      Eigen::VectorXd* v_error)
{
    *v_error = desired_trajectory.getVel() - odometry.getVel();
}

inline void LimitTensions(Eigen::Vector4d* tensions)
{
    Eigen::Vector4d max_tensions(10.0, 10.0, 10.0, 10.0);
    *tensions = tensions->cwiseMax(Eigen::VectorXd::Zero(tensions->rows()));
    *tensions = tensions->cwiseMin(max_tensions);
}

inline Eigen::VectorXd sgn(Eigen::VectorXd& sliding_surface)
{
    Eigen::VectorXd sgn_s = Eigen::VectorXd::Zero(6);

    double alpha = 5;
    for (unsigned int i=0;i<sliding_surface.size();++i)
        sgn_s(i) = tanh(alpha * sliding_surface(i));

    return sgn_s;
}

} //namespace system_commander

#endif //PYRAMID_CENTRAL_COMMON_H
