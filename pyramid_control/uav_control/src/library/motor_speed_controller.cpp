#include "uav_control/motor_speed_controller.h"

namespace motor_speed_control
{

MotorSpeedController::MotorSpeedController()
    :initialized_params_(false)
{
    InitializeParameters();
}

MotorSpeedController::~MotorSpeedController(){ }

void MotorSpeedController::InitializeParameters()
{
    //calculate allocation matrix from vehicle parameters
    calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));

    Eigen::Matrix4d I;
    I.setZero();
    I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
    I(3, 3) = 1;

    //calculate inverse allocation matrix
    thrust_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);

    thrust_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
        * (controller_parameters_.allocation_matrix_
        * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;

    initialized_params_ = true;
}

void MotorSpeedController::SetThrustMsg(const EigenWrenchStamped& thrust)
{
  thrust_ = thrust;
}

void MotorSpeedController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const
{
    assert(initialized_params_);

    //Update vector for rotor number
    rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());


    Eigen::Vector4d limited_thrust;
    limited_thrust.block<3, 1>(0, 0) = thrust_.getTorque();
    limited_thrust(3) = thrust_.getZforce();

    *rotor_velocities = thrust_to_rotor_velocities_ * limited_thrust;
    *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
    *rotor_velocities = rotor_velocities->cwiseSqrt();
}

} //motor_speed_control
