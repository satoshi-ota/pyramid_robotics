#include "pyramid_central/system_commander.h"

namespace system_commander
{

SystemCommander::SystemCommander(){ }

SystemCommander::~SystemCommander(){ }

void SystemCommander::UpdateParams()
{
    CalculateRotationMatrix(odometry_.orientation_EO, system_parameters_.rotation_matrix_);
    CalculateSpatialInertiaMatrix(rotor_configuration_, &rotation_matrix_, &spatial_mass_matrix_);
    CalculateCentrifugalCoriolisMatrix();
    CalculateJacobian();
}

void SystemCommander::SetDesiredTrajectry(const EigenMultiDOFJointTrajectry& trajectory)
{
    desired_trajectry_ = trajectory;
}

void SystemCommander::SetFeedbackOdometry(const EigenOdometry& odometry)
{
    odometry_ = odometry;
}

void CalculateSystemDynamicsParameters()
{
    system_parameters_.CalculateSpatialInertiaMatrix();
    system_parameters_.CalculateCentrifugalCoriolisMatrix();
    system_parameters_.CalculateJacobian();
}

} //namespace system_commander
