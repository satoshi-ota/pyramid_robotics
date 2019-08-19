#include "pyramid_central/system_commander.h"

namespace system_commander
{

SystemCommander::SystemCommander()
{

}

SystemCommander::~SystemCommander(){ }

void SystemCommander::UpdateAnchorPos()
{
    //dynamic anchor_position
    /*
    for (Tether& tether : tether_configuration_.tethers)
    {
        tether.anchor_position =
    }
    */
}

void SystemCommander::UpdateTetherDirections()
{
    for (Tether& tether : system_parameters_.tether_configuration_.tethers)
    {
        tether.direction = tether.anchor_position - odometry_.position_EO
                           - rotation_matrix_ * tether.mounting_pos;
    }
}

void SystemCommander::UpdateDynamicParams()
{

    CalculateRotationMatrix(odometry_.orientation_EO, &rotation_matrix_);

    CalculateGlobalInertia(system_parameters_.inertia_, rotation_matrix_, &global_inertia_);

    CalculateAngularMappingMatrix(odometry_.orientation_EO, &angular_mapping_matrix_);

    CalculateJacobian(system_parameters_.tether_configuration_, rotation_matrix_, jacobian_);


    CalculateSpatialInertiaMatrix(system_parameters_, global_inertia_,
                                  angular_mapping_matrix_, &spatial_mass_matrix_);

    CalculateCentrifugalCoriolisMatrix(odometry_.angular_velocity_EO, global_inertia_,
                                       angular_mapping_matrix_, derivative_angular_mapping_matrix_, &centrifugal_coriolis_matrix_);

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
    input_acc_.resize(6);
    Eigen::VectorXd x_delta = Eigen::VectorXd::Zero(6);
    CalculatePosAttDelta(odometry_, desired_position_, desired_orientarion_, &x_delta);
    Eigen::VectorXd v_delta = Eigen::VectorXd::Zero(6);
    CalculateVelocityDelta(odometry_, desired_velocity_, desired_angular_velocity_, &v_delta);
    Eigen::VectorXd acc_desired = Eigen::VectorXd::Zero(6);

    acc_desired.block<3, 1>(0, 0) = desired_acceleration_;
    acc_desired.block<3, 1>(3, 0) = desired_angular_acceleration_;

    input_acc_ = acc_desired + system_parameters_.K_d_ * v_delta
                 + system_parameters_.K_p_ * x_delta;
}

void SystemCommander::CalculateConrolVariable(Eigen::VectorXd* control_input)
{
    control_input->resize(10);
    CalculateWrench(system_parameters_, odometry_, spatial_mass_matrix_,
                    centrifugal_coriolis_matrix_, input_acc_, &wrench_);

    Eigen::Matrix<double, 6, 6> R;
    R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    R.block<3, 3>(3, 3) = rotation_matrix_;

    Eigen::Matrix<double, 6, 10> B;
    B.block<6, 4>(0, 0) = (jacobian_ * R).transpose();
    B.block<3, 3>(0, 4) = rotation_matrix_;
    B.block<3, 3>(3, 7) = Eigen::Matrix3d::Identity();

    *control_input = B.transpose() * (B * B.transpose()).inverse() * wrench_;
}


} //namespace system_commander
