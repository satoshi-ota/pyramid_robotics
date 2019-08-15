#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include <ros/ros.h>

#include <uav_control/conversions.h>
#include <uav_control/parameters.h>
#include <uav_control/get_params.h>

#include <geometry_msgs/WrenchStamped.h>
#include <pyramid_msgs/pyramid_eigen_msgs.h>



namespace motor_speed_control
{

class MotorSpeedControllerParameters
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LeePositionControllerParameters()
        : position_gain_(kDefaultPositionGain),
    velocity_gain_(kDefaultVelocityGain),
    attitude_gain_(kDefaultAttitudeGain),
    angular_rate_gain_(kDefaultAngularRateGain){
        calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
    }

    Eigen::Matrix4Xd allocation_matrix_;
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d attitude_gain_;
    Eigen::Vector3d angular_rate_gain_;
    RotorConfiguration rotor_configuration_;
};

class MotorSpeedController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW //eigen macro

    MotorSpeedController();
    ~MotorSpeedController();

    void InitializeParameters();

    void setThrustMsg(const EigenWrenchStamped& thrust);
    void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

    VehicleParameters vehicle_parameters_;

private:
    //general
    bool initialized_params_;
    EigenWrenchStamped thrust_;

    //inverse allocation matrix
    Eigen::MatrixX4d thrust_to_rotor_velocities_;

};

} //motor_speed_control

#endif //MOTOR_SPEED_CONTROLLER_H
