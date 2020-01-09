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

const double kDt = 0.1;
Eigen::Matrix3d DT = Eigen::Vector3d(kDt, kDt, kDt).asDiagonal();

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

    Eigen::VectorXd xEst_;
    Eigen::MatrixXd PEst_;

    Eigen::Vector3d beta_;

    Eigen::MatrixXd kA_;
    Eigen::MatrixXd kBu_;
    Eigen::MatrixXd kB_;
    Eigen::MatrixXd kC_;

    Eigen::MatrixXd kQ_;
    Eigen::MatrixXd kR_;
};

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_OBSERVER_H
