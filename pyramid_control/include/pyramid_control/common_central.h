#ifndef PYRAMID_CONTROL_COMMON_H
#define PYRAMID_CONTROL_COMMON_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <pyramid_msgs/common.h>
#include <pyramid_msgs/pyramid_eigen_msgs.h>
#include <geometry_msgs/WrenchStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "pyramid_control/parameters.h"

namespace pyramid_control
{

inline void skewMatrixFromVector(Eigen::Vector3d& vector, Eigen::Matrix3d* skew_matrix)
{
  *skew_matrix <<          0,-vector.z(),  vector.y(),
                  vector.z(),          0, -vector.x(),
                 -vector.y(), vector.x(),           0;
}

inline void calcRotMatrix(const Eigen::Quaterniond& orientation, Eigen::Matrix3d* rotMatrix)
{
    *rotMatrix = orientation.toRotationMatrix();
}

inline void calcGlobalInertia(const Eigen::Matrix3d& inertia,
                              const Eigen::Matrix3d& rotMatrix,
                                    Eigen::Matrix3d* globalInertia)
{
    *globalInertia = rotMatrix * inertia * rotMatrix.transpose();
}

inline void calcToOmage(const Eigen::Quaterniond& orientation, Eigen::Matrix3d* toOmega)
{
    Eigen::Vector3d angles;
    pyramid_msgs::getEulerAnglesFromQuaternion(orientation, &angles);

    *toOmega << 1,  0,              -sin(angles(1)),
                0,  cos(angles(0)),  cos(angles(1))*sin(angles(0)),
                0, -sin(angles(0)),  cos(angles(1))*cos(angles(0));
}

inline void calcToOmageDot(const Eigen::Quaterniond& orientation, Eigen::Matrix3d* toOmega_dot)
{
    Eigen::Vector3d angles;
    pyramid_msgs::getEulerAnglesFromQuaternion(orientation, &angles);

    *toOmega_dot
        << 0,  0,              -cos(angles(1)),
           0, -sin(angles(0)), -sin(angles(1))*sin(angles(0))+cos(angles(1))*sin(angles(0)),
           0, -cos(angles(0)), -sin(angles(1))*cos(angles(0))-cos(angles(1))*sin(angles(0));
}

inline void calcMassMatrix(const SystemParameters& system_parameters,
                                          const Eigen::Matrix3d& globalInertia,
                                          const Eigen::Matrix3d& toOmega,
                                                Eigen::MatrixXd* massMatrix)
{
    Eigen::Matrix<double, 6, 6> Mat = Eigen::MatrixXd::Zero(6, 6);

    Eigen::Matrix3d kI = Eigen::Matrix3d::Identity();
    Mat.block<3, 3>(0, 0) = system_parameters.mass_ * kI.array();
    Mat.block<3, 3>(3, 3) = toOmega.transpose() * globalInertia * toOmega;

    *massMatrix = Mat;
}

inline void calcCoriolisMatrix(const Eigen::Vector3d& angular_velocity,
                               const Eigen::Matrix3d& globalInertia,
                               const Eigen::Matrix3d& toOmega,
                               const Eigen::Matrix3d& toOmega_dot,
                                     Eigen::MatrixXd* coriolisMatrix)
{
    Eigen::Matrix<double, 6, 6> Mat = Eigen::MatrixXd::Zero(6, 6);

    Eigen::Matrix3d skewMatrix;
    Eigen::Vector3d omega = angular_velocity;
    skewMatrixFromVector(omega, &skewMatrix);

    Mat.block<3, 3>(3, 3) = toOmega.transpose() * globalInertia * toOmega_dot
                          + toOmega.transpose() * skewMatrix * globalInertia * toOmega;

    *coriolisMatrix = Mat;
}

inline void calcJacobian(const TetherConfiguration& tether_configuration,
                         const Eigen::Matrix3d& rotMatrix,
                               Eigen::MatrixXd* jacobian)
{
    assert(jacobian != NULL);

    jacobian->resize(8, 6);
    jacobian->setZero();

    Eigen::Matrix<double, 6, 8> J = Eigen::MatrixXd::Zero(6, 8);

    unsigned int i = 0;
    for (const PseudoTether& pseudo_tether : tether_configuration.pseudo_tethers)
    {
        J.block<3, 1>(0, i) =  pseudo_tether.direction;
        J.block<3, 1>(3, i) = -pseudo_tether.direction.cross(rotMatrix*pseudo_tether.attach_pos);

        ++i;
    }
    *jacobian = J.transpose();
}

inline void calcJacobianTilde(const Eigen::Matrix3d& toOmega,
                                   const Eigen::MatrixXd& jacobian,
                                         Eigen::MatrixXd* jacobian_tilde)
{
    jacobian_tilde->resize(4, 6);
    jacobian_tilde->setZero();

    Eigen::Matrix<double, 6, 6> S1 = Eigen::MatrixXd::Zero(6, 6);

    S1.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    S1.block<3, 3>(3, 3) = toOmega;

    *jacobian_tilde = jacobian * S1;
}

inline void calcPosAttDelta(const pyramid_msgs::EigenOdometry& odometry,
                            const pyramid_msgs::EigenMultiDOFJointTrajectory& trajectory,
                                         Eigen::VectorXd* xError)
{
    *xError = trajectory.getPosAtt() - odometry.getPosAtt();
}

inline void calcVelDelta(const pyramid_msgs::EigenOdometry& odometry,
                              const pyramid_msgs::EigenMultiDOFJointTrajectory& trajectory,
                                           Eigen::VectorXd* vError)
{
    *vError = trajectory.getVel() - odometry.getGrobalVel();
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

    double alpha = 10;
    for (unsigned int i=0;i<sliding_surface.size();++i)
        sgn_s(i) = tanh(alpha * sliding_surface(i));

    return sgn_s;
}

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_COMMON_H
