#ifndef PYRAMID_CONTROL_OBSERVER_H
#define PYRAMID_CONTROL_OBSERVER_H

#include <Eigen/QR>
#include <iostream>

#include <ros/ros.h>
#include <pyramid_msgs/conversions.h>
#include <pyramid_msgs/pyramid_eigen_msgs.h>

#include "pyramid_control/common.h"
#include "pyramid_control/configurations.h"

using namespace std;

namespace pyramid_control
{

class Observer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Observer(SystemParameters* system_parameters);
    ~Observer();

    void estimateDisturbance(const Eigen::VectorXd& wrench,
                                   Eigen::VectorXd* disturbance);

    SystemParameters *system_parameters_ = new SystemParameters();

private:
    Eigen::Matrix3d torqueOBSGain_;

    Eigen::VectorXd disturbance_;

    Eigen::VectorXd xEst_;
    Eigen::MatrixXd PEst_;

    Eigen::Vector3d beta_;

    Eigen::MatrixXd kA_;
    Eigen::MatrixXd kBu_;
    Eigen::MatrixXd kB_;
    Eigen::MatrixXd kC_;
    Eigen::VectorXd kG_;

    Eigen::MatrixXd kQ_;
    Eigen::MatrixXd kR_;

    void fEst(const Eigen::VectorXd& u);
    void tEst(const Eigen::VectorXd& u);
};

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_OBSERVER_H
