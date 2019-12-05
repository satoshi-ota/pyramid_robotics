#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include "uav_control/common.h"
#include "uav_control/parameters.h"
#include "uav_control/configurations.h"

namespace motor_speed_control
{

class MotorSpeedControllerParameters
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MotorSpeedControllerParameters()
    {
        calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
    }

    Eigen::Matrix4Xd allocation_matrix_;
    RotorConfiguration rotor_configuration_;
};

class MotorSpeedController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW //eigen macro

    MotorSpeedController();
    ~MotorSpeedController();

    void InitializeParameters();

    void SetThrust(const EigenWrenchStamped& thrust);
    void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

    MotorSpeedControllerParameters controller_parameters_;
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
