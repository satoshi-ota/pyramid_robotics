#ifndef PYRAMID_CONTROL_SLIDING_MODE_CONTROLLER_H
#define PYRAMID_CONTROL_SLIDING_MODE_CONTROLLER_H

#include <Eigen/QR>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <pyramid_msgs/conversions.h>
#include <pyramid_msgs/pyramid_eigen_msgs.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "pyramid_control/common.h"
#include "pyramid_control/common_central.h"
#include "pyramid_control/configurations.h"

using namespace std;

namespace pyramid_control
{

class SlidingModeController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SlidingModeController();
    ~SlidingModeController();

    void updateModelConfig();
    void calcThrust();

    SystemParameters system_parameters_;

    inline void setTrajectory(
        const pyramid_msgs::EigenMultiDOFJointTrajectory& trajectory){trajectory_ = trajectory;};
    inline void setOdometry(const pyramid_msgs::EigenOdometry& odometry){odometry_ = odometry;};
    inline Eigen::VectorXd getWrench(){return wrench_;};
    inline Eigen::MatrixXd getJacobian(){return jacobian_;};
    inline Eigen::Matrix3d getRotationMatrix(){return rotMatrix_;};
    inline Eigen::Matrix3d getToOmegaMatrix(){return toOmega_dot_;};

    inline pyramid_msgs::EigenThrust getThrust(){return thrust_;};

private:
    Eigen::Matrix3d rotMatrix_;
    Eigen::Matrix3d globalInertia_;
    Eigen::Matrix3d toOmega_;
    Eigen::Matrix3d toOmega_dot_;

    Eigen::MatrixXd massMatrix_;
    Eigen::MatrixXd coriolisMatrix_;
    Eigen::MatrixXd jacobian_;

    Eigen::VectorXd slidingSurface_;
    Eigen::VectorXd xError_;
    Eigen::VectorXd vError_;
    Eigen::VectorXd wrench_;

    pyramid_msgs::EigenMultiDOFJointTrajectory trajectory_;
    pyramid_msgs::EigenOdometry odometry_;
    pyramid_msgs::EigenThrust thrust_;
};

}

#endif //PYRAMID_CONTROL_SLIDING_MODE_CONTROLLER_H
