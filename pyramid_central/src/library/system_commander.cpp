#include "pyramid_central/system_commander.h"
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl

namespace system_commander
{

SystemCommander::SystemCommander()
    :rotation_matrix_(Eigen::Matrix3d::Zero()),
     global_inertia_(Eigen::Matrix3d::Zero()),
     angular_mapping_matrix_(Eigen::Matrix3d::Zero()),
     derivative_angular_mapping_matrix_(Eigen::Matrix3d::Zero()),
     spatial_mass_matrix_(Eigen::MatrixXd::Zero(6, 6)),
     centrifugal_coriolis_matrix_(Eigen::MatrixXd::Zero(6, 6)),
     wrench_(Eigen::VectorXd::Zero(6)),
     input_acceleration_(Eigen::VectorXd::Zero(6)){ }

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
        tether.direction = (tether.anchor_position - odometry_.position_EO
                            - rotation_matrix_ * tether.mounting_pos).normalized();
    }
}

void SystemCommander::UpdateDynamicParams()
{

    CalculateRotationMatrix(odometry_.orientation_EO, &rotation_matrix_);

    CalculateGlobalInertia(system_parameters_.inertia_, rotation_matrix_, &global_inertia_);

    CalculateAngularMappingMatrix(odometry_.orientation_EO, &angular_mapping_matrix_);

    CalculateJacobian(system_parameters_.tether_configuration_, rotation_matrix_, &jacobian_);

    CalculateSpatialInertiaMatrix(system_parameters_, global_inertia_,
                                  angular_mapping_matrix_, &spatial_mass_matrix_);

    CalculateCentrifugalCoriolisMatrix(odometry_.angular_velocity_EO, global_inertia_,
                                       angular_mapping_matrix_, derivative_angular_mapping_matrix_, &centrifugal_coriolis_matrix_);
}

void SystemCommander::SetDesiredTrajectory(const EigenMultiDOFJointTrajectory& trajectory)
{
    desired_trajectory_ = trajectory;
    ROS_INFO("Set desired trajectory: position[%f, %f, %f].",
             desired_trajectory_.position_ET.x(),
             desired_trajectory_.position_ET.y(),
             desired_trajectory_.position_ET.z());
}

void SystemCommander::SetFeedbackOdometry(const EigenOdometry& odometry)
{
    odometry_ = odometry;
}

void SystemCommander::CalculateInputAcc()
{
    Eigen::VectorXd x_delta = Eigen::VectorXd::Zero(6);
    CalculatePosAttDelta(odometry_, desired_trajectory_, &x_delta);
    Eigen::VectorXd v_delta = Eigen::VectorXd::Zero(6);
    CalculateVelocityDelta(odometry_, desired_trajectory_, &v_delta);
    Eigen::VectorXd acc_desired = Eigen::VectorXd::Zero(6);

    acc_desired.block<3, 1>(0, 0) = desired_trajectory_.acceleration_ET;
    acc_desired.block<3, 1>(3, 0) = desired_trajectory_.angular_acceleration_ET;

    input_acceleration_ = acc_desired + system_parameters_.K_d_ * v_delta
                          + system_parameters_.K_p_ * x_delta;
}

void SystemCommander::CalculateConrolVariable()
{
    CalculateWrench(system_parameters_, odometry_, spatial_mass_matrix_,
                    centrifugal_coriolis_matrix_, input_acceleration_, &wrench_);

    thrust_.force.setZero();
    thrust_.force(2) = (rotation_matrix_.inverse() * wrench_.block<3, 1>(0, 0)).z();

    Eigen::MatrixXd jacobian_tilde;

    CalculateJacobianTilde(angular_mapping_matrix_, jacobian_, &jacobian_tilde);

    Eigen::Vector3d wrench_delta = wrench_.block<3, 1>(0, 0) - rotation_matrix_ * thrust_.force;

    Eigen::FullPivLU<Eigen::MatrixXd> lu(jacobian_tilde.block<4, 3>(0, 0).transpose());
    tensions_ = lu.solve(wrench_delta);

    LimitTensions(&tensions_);
    PRINT_MAT(wrench_);
    thrust_.torque
        = wrench_.block<3, 1>(3, 0) - (jacobian_tilde.transpose() * tensions_).block<3, 1>(3, 0);
}


} //namespace system_commander
