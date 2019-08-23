#ifndef PYRAMID_CENTRAL_SLIDING_MODE_CONTROLLER_H
#define PYRAMID_CENTRAL_SLIDING_MODE_CONTROLLER_H

#include <Eigen/QR>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
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

    void SetDesiredTrajectory(const EigenMultiDOFJointTrajectory& trajectory);
    void SetFeedbackOdometry(const EigenOdometry& odometry);

    void UpdateDynamicParams();
    void CalculateSlidingSurface();
    void CalculateThrust();

    //from parameters.h
    SystemParameters system_parameters_;

    inline EigenThrust getThrust(){return thrust_;};

private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //variable
    Eigen::Matrix3d rotation_matrix_; //R
    Eigen::Matrix3d global_inertia_; //I_g
    Eigen::Matrix3d angular_mapping_matrix_; //S
    Eigen::Matrix3d derivative_angular_mapping_matrix_; //S

    Eigen::MatrixXd spatial_mass_matrix_; //M
    Eigen::MatrixXd centrifugal_coriolis_matrix_; //C

    Eigen::MatrixXd jacobian_; //J

    Eigen::VectorXd sliding_surface_;
    Eigen::VectorXd x_delta_;
    Eigen::VectorXd v_delta_;
    Eigen::VectorXd wrench_; //f
    Eigen::VectorXd input_acceleration_;

    //goal
    EigenMultiDOFJointTrajectory desired_trajectory_;

    //feedback
    EigenOdometry odometry_;

    //countoller output
    EigenThrust thrust_;

};

}

#endif //PYRAMID_CENTRAL_SLIDING_MODE_CONTROLLER_H
