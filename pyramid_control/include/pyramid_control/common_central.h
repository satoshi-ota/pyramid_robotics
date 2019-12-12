#ifndef PYRAMID_CONTROL_CENTRAL_COMMON_H
#define PYRAMID_CONTROL_CENTRAL_COMMON_H

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

inline void skewMatrixFromVector(Eigen::Vector3d& vector, Eigen::Matrix3d* skewMatrix)
{
  *skewMatrix <<          0,-vector.z(),  vector.y(),
                 vector.z(),          0, -vector.x(),
                -vector.y(), vector.x(),           0;
}

inline void calcToOmage(const Eigen::Quaterniond& orientation, Eigen::Matrix3d* toOmega)
{
    Eigen::Vector3d rpy;
    pyramid_msgs::getEulerAnglesFromQuaternion(orientation, &rpy);

    *toOmega << 1,  0,           -sin(rpy(1)),
                0,  cos(rpy(0)),  cos(rpy(1))*sin(rpy(0)),
                0, -sin(rpy(0)),  cos(rpy(1))*cos(rpy(0));
}

inline void calcToOmageDot(const Eigen::Quaterniond& orientation, Eigen::Matrix3d* toOmega_dot)
{
    Eigen::Vector3d rpy;
    pyramid_msgs::getEulerAnglesFromQuaternion(orientation, &rpy);

    *toOmega_dot
        << 0,  0,           -cos(rpy(1)),
           0, -sin(rpy(0)), -sin(rpy(1))*sin(rpy(0))+cos(rpy(1))*sin(rpy(0)),
           0, -cos(rpy(0)), -sin(rpy(1))*cos(rpy(0))-cos(rpy(1))*sin(rpy(0));
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

    int tether_num = tether_configuration.pseudo_tethers.size();

    jacobian->resize(tether_num, 6);
    jacobian->setZero();

    Eigen::MatrixXd J;
    J.resize(6, tether_num);
    J.setZero();

    unsigned int i = 0;
    for (const PseudoTether& pseudo_tether : tether_configuration.pseudo_tethers)
    {
        J.block<3, 1>(0, i) = -pseudo_tether.direction;
        J.block<3, 1>(3, i) =  pseudo_tether.direction.cross(rotMatrix*pseudo_tether.attach_pos);

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

inline Eigen::MatrixXd calcRotorMatrix(const Eigen::Matrix3d& rotMatrix)
{
    double rot13 = rotMatrix(0, 2);
    double rot23 = rotMatrix(1, 2);
    double rot33 = rotMatrix(2, 2);

    Eigen::MatrixXd Mat;
    Mat.resize(6, 4);
    Mat.setZero();

    Mat << rot13, 0, 0, 0,
           rot23, 0, 0, 0,
           rot33, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;

    return Mat;
}

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_CENTRAL_COMMON_H
