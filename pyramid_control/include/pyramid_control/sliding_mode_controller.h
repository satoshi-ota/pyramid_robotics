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
#include "pyramid_control/configurations.h"

using namespace std;

namespace pyramid_control
{

class SlidingModeController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SlidingModeController(SystemParameters* system_parameters);
    ~SlidingModeController();

    void updateModelConfig();
    void calcThrust(Eigen::VectorXd* wrench);

    SystemParameters *system_parameters_ = new SystemParameters();

    inline void setTrajectory(const pyramid_msgs::EigenMultiDOFJointTrajectory& trajectory){
        trajectory_ = trajectory;
    };

private:
    pyramid_msgs::EigenMultiDOFJointTrajectory trajectory_;
};

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_SLIDING_MODE_CONTROLLER_H
