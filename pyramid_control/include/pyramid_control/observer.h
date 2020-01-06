#ifndef PYRAMID_CONTROL_OBSERVER_H
#define PYRAMID_CONTROL_OBSERVER_H

#include <Eigen/QR>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <pyramid_msgs/conversions.h>
#include <pyramid_msgs/pyramid_eigen_msgs.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "pyramid_control/common.h"
#include "pyramid_control/configurations.h"

using namespace std;

namespace pyramid_control
{

const double kdt = 0.1;
Eigen::Matrix3d DT = Eigen::Vector3d(kdt, kdt, kdt).asDiagonal();

class Observer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Observer(SystemParameters* system_parameters);
    ~Observer();

    void estimateDisturbance(const Eigen::VectorXd& wrench);

    SystemParameters *system_parameters_ = new SystemParameters();

private:
    Eigen::Vector3d forceDisturbance_;
    Eigen::Vector3d torqueDisturbance_;
    Eigen::Matrix3d forceOBSGain_;
    Eigen::Matrix3d torqueOBSGain_;

    Eigen::Matrix3d covQ, covR;

    Eigen::VectorXd xEst_;

    Eigen::Vector3d beta_;

    Eigen::MatrixXd matA_;
    Eigen::MatrixXd matB_;
    Eigen::MatrixXd matC_;

    Eigen::MatrixXd matQ_;
    Eigen::MatrixXd matR_;

    Eigen::MatrixXd PEst_;
};

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_OBSERVER_H
