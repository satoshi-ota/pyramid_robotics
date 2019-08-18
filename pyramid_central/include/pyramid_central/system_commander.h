#ifndef PYRAMID_CENTRAL_SYSTEM_COMMANDER_H
#define PYRAMID_CENTRAL_SYSTEM_COMMANDER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "pyramid_central/common.h"
#include "pyramid_central/configurations.h"

namespace system_commander
{

class SystemCommander
{
public:
    SystemCommander(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~SystemCommander();

    void SetDesiredTrajectory(const EigenMultiDOFJointTrajectory& trajectory);
    void SetFeedbackOdometry(const EigenOdometry& odometry);

    void UpdateTetherDirections(const EigenOdometry& odometry);
    void UpdateDynamicParams();

    void CalculateInputAcc();
    void CalculateConrolVariable();

    //from parameters.h
    SystemParameters system_parameters_;

private: //member data
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //variable
    Eigen::Matrix3d rotation_matrix_; //R
    Eigen::Matrix3d global_inertia_; //I_g
    Eigen::Matrix3d angular_mapping_matrix_; //S
    Eigen::Matrix3d derivative_angular_mapping_matrix_; //S

    Eigen::Matrix<double, 6, 6> spatial_mass_matrix_; //M
    Eigen::Matrix<double, 6, 6> centrifugal_coriolis_matrix_; //C

    Eigen::MatrixXd jacobian_; //J

    Eigen::VectorXd wrench_; //f

    Eigen::VectorXd control_input_; //u

    //linear
    Eigen::Vector3d desired_position_;
    Eigen::Vector3d desired_velocity_;
    Eigen::Vector3d desired_acceleration_;

    //angular
    Eigen::Quaterniond desired_quaternion_;
    Eigen::Vector3d desired_angular_velocity_;
    Eigen::Vector3d desired_angular_acceleration_;

    //goal
    trajectory_msgs::MultiDOFJointTrajectory desired_trajectory_;

    //feedback
    EigenOdometry odometry_;
    sensor_msgs::Imu imu_;

    //anchor pos
    Eigen::VectorXd anchor_positions_;


};

} //namespace system_commander

#endif //PYRAMID_CENTRAL_SYSTEM_COMMANDER_H
