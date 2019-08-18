#include "pyramid_central/system_commander.h"

namespace system_commander
{

SystemCommander::SystemCommander(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh){ }

SystemCommander::~SystemCommander(){ }

void SystemCommander::UpdateTetherDirections()
{
    CalculateTetherDirections(system_parameters_.tether_configuration_,
                              odometry_, rotation_matrix_, anchor_positions_);
}

void SystemCommander::UpdateDynamicParams()
{

    CalculateRotationMatrix(odometry_.orientation_EO, &rotation_matrix_);

    CalculateGlobalInertia(vehicle_parameters_, rotation_matrix_, &global_inertia_);

    CalculateAngularMappingMatrix(orientation_,  &angular_mapping_matrix_);

    CalculateJacobian(vehicle_parameters_, tether_states_, rotation_matrix_, &jacobian_);

    CalculateSpatialInertiaMatrix(vehicle_parameters_, global_inertia_,
                                  angular_mapping_matrix_, &spatial_mass_matrix_);

    CalculateCentrifugalCoriolisMatrix(odometry_.angular_velocity_EO,
                                       angular_mapping_matrix_, &centrifugal_coriolis_matrix_);

}

void SystemCommander::SetDesiredTrajectory(const EigenMultiDOFJointTrajectory& trajectory)
{
    desired_trajectory_ = trajectory;
}

void SystemCommander::SetFeedbackOdometry(const EigenOdometry& odometry)
{
    odometry_ = odometry;
}

void SystemCommander::CalculateInputAcc()
{
    Eigen::VectorXd x_delta = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd v_delta = Eigen::VectorXd::Zero(6);
    input_acc = acc_d + K_d * v_delta + K_p * x_delta;
}

void SystemCommander::CalculateConrolVariable()
{
    CalculateWrench(wrench_);

    Eigen::Matrix<double, 6, 6> R;
    R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    R.block<3, 3>(3, 3) = rotation_matrix;

    Eigen::Matrix<double, 6, 10> B;
    B.block<6, 4>(0, 0) = (jacobian * R).transpose();
    B.block<3, 3>(0, 4) = rotation_matrix;
    B.block<3, 3>(3, 7) = Eigen::Matrix3d::Identity();

    control_input = B.transpose() * (B * B.transpose()).inverse() * wrench_;
}


} //namespace system_commander
