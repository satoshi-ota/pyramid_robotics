#ifndef PYRAMID_CENTRAL_SYSTEM_COMMANDER_H
#define PYRAMID_CENTRAL_SYSTEM_COMMANDER_H

#include <ros/ros.h>
#include <geometry_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <trajectry_msgs/MultiDOFJointTrajectry.h>

#include "pyramid_central/common.h"
#include "pyramid_central/configurations.h"

namespace system_commander
{

class SystemParameters
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SystemParameters(){ }
    ~SystemParameters(){ }

    Eigen::Matrix3d rotation_matrix_;
    Eigen::Matrix3d global_inertia_;

    Eigen::Matrix6d spatial_mass_matrix_;
    Eigen::Matrix6d centrifugal_coriolis_matrix_;
    Eigen::Matrix6d angular_derivative_matrix_;
    Eigen::MatrixXd jacobian_;

    TetherConfiguration tether_configuration_;

    RotorConfiguration rotor_configuration_;
};

class SystemCommander
{
public:
    SystemCommander(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~SystemCommander();

    void UpdateParams();

    void SetDesiredTrajectry(const EigenMultiDOFJointTrajectry& trajectory);
    void SetFeedbackOdometry(const EigenOdometry& odometry);

    void CalculateSystemDynamicsParameters();
    void CalculateAcceleration();
    void CalculateTensions();

    SystemParameters system_parameters_;
    VehicleParameters vehicle_parameters_;


private: //member data
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //parameters
    SystemParameters system_parameters_;
    VehicleParameters vehicle_parameters_;

    //linear
    Eigen::Vector3d desired_position_;
    Eigen::Vector3d desired_velocity_;
    Eigen::Vector3d desired_acceleration_;

    //angular
    Eigen::Quaterniond desired_quaternion_;
    Eigen::Vector3d desired_angular_velocity_;
    Eigen::Vector3d desired_angular_acceleration_;

    //goal
    trajectry_msgs::MultiDOFJointTrajectry desired_trajectry_;

    //feedback
    EigenOdometry odometry_;
    sensor_msgs::Imu imu_;


};

} //namespace system_commander

#endif //PYRAMID_CENTRAL_SYSTEM_COMMANDER_H
