#include "pyramid_control/motor_speed_controller.h"

namespace pyramid_control
{

MotorSpeedController::MotorSpeedController()
    :initialized_params_(false)
{
    InitializeParameters();
}

MotorSpeedController::~MotorSpeedController(){ }

void MotorSpeedController::InitializeParameters()
{
    calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));

    thrust_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);

    thrust_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
        * (controller_parameters_.allocation_matrix_
        * controller_parameters_.allocation_matrix_.transpose()).inverse();

    initialized_params_ = true;
}

void MotorSpeedController::SetThrust(const EigenWrenchStamped& thrust)
{
  thrust_ = thrust;
}

void MotorSpeedController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const
{
    assert(initialized_params_);

    rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());

    Eigen::Vector4d limited_thrust;
    limited_thrust.block<3, 1>(0, 0) = thrust_.getTorque();
    limited_thrust(3) = thrust_.getZforce();

    *rotor_velocities = thrust_to_rotor_velocities_ * limited_thrust;
    *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
    *rotor_velocities = rotor_velocities->cwiseSqrt();
}

} //namespace pyramid_control
