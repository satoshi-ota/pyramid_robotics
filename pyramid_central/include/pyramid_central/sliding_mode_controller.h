#ifndef PYRAMID_CENTRAL_SLIDING_MODE_CONTROLLER_H
#define PYRAMID_CENTRAL_SLIDING_MODE_CONTROLLER_H

#include <Eigen/QR>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <pyramid_msgs/conversions.h>
#include <pyramid_msgs/pyramid_eigen_msgs.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "pyramid_central/common.h"
#include "pyramid_central/configurations.h"

using namespace std;

namespace system_commander
{

class SlidingModeController
{
public:
    SlidingModeController();
    ~SlidingModeController();

    void SetDesiredTrajectory(const pyramid_msgs::EigenMultiDOFJointTrajectory& trajectory);
    void SetFeedbackOdometry(const pyramid_msgs::EigenOdometry& odometry);

    void UpdateTetherDirections();
    void UpdateDynamicParams();
    void CalculateSlidingSurface();
    void CalculateThrust();

    //from parameters.h
    SystemParameters system_parameters_;

    inline Eigen::VectorXd getWrench(){return wrench_;};
    inline Eigen::MatrixXd getJacobian(){return jacobian_;};
    inline Eigen::Matrix3d getRotationMatrix(){return rotation_matrix_;};
    inline Eigen::Matrix3d getToOmegaMatrix(){return angular_mapping_matrix_;};

    inline pyramid_msgs::EigenThrust getThrust(){return thrust_;};

private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //variable
    Eigen::Matrix3d rotation_matrix_;
    Eigen::Matrix3d global_inertia_;
    Eigen::Matrix3d angular_mapping_matrix_;
    Eigen::Matrix3d derivative_angular_mapping_matrix_;

    Eigen::MatrixXd spatial_mass_matrix_;
    Eigen::MatrixXd centrifugal_coriolis_matrix_;
    Eigen::MatrixXd jacobian_;

    Eigen::VectorXd sliding_surface_;
    Eigen::VectorXd x_error_;
    Eigen::VectorXd v_error_;
    Eigen::VectorXd wrench_;

    pyramid_msgs::EigenMultiDOFJointTrajectory desired_trajectory_;
    pyramid_msgs::EigenOdometry odometry_;
    pyramid_msgs::EigenThrust thrust_;
};

}

#endif //PYRAMID_CENTRAL_SLIDING_MODE_CONTROLLER_H
