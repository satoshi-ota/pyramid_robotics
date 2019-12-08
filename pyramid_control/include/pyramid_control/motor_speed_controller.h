#ifndef PYRAMID_CONTROL_MOTOR_SPEED_CONTROLLER_H
#define PYRAMID_CONTROL_MOTOR_SPEED_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include "pyramid_control/common.h"
#include "pyramid_control/parameters.h"
#include "pyramid_control/configurations.h"

namespace pyramid_control
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
    bool initialized_params_;
    EigenWrenchStamped thrust_;

    Eigen::MatrixX4d thrust_to_rotor_velocities_;

};

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_MOTOR_SPEED_CONTROLLER_H
