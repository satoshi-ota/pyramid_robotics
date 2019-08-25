#include "pyramid_central/sliding_mode_controller.h"
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl

namespace system_commander
{

SlidingModeController::SlidingModeController()
    :rotation_matrix_(Eigen::Matrix3d::Zero()),
     global_inertia_(Eigen::Matrix3d::Zero()),
     angular_mapping_matrix_(Eigen::Matrix3d::Zero()),
     derivative_angular_mapping_matrix_(Eigen::Matrix3d::Zero()),
     spatial_mass_matrix_(Eigen::MatrixXd::Zero(6, 6)),
     centrifugal_coriolis_matrix_(Eigen::MatrixXd::Zero(6, 6)),
     wrench_(Eigen::VectorXd::Zero(6)),
     sliding_surface_(Eigen::VectorXd::Zero(6)),
     x_error_(Eigen::VectorXd::Zero(6)),
     v_error_(Eigen::VectorXd::Zero(6)){ }

SlidingModeController::~SlidingModeController(){ }

void SlidingModeController::UpdateTetherDirections()
{
    for (Tether& tether : system_parameters_.tether_configuration_.tethers)
    {
        tether.direction = (tether.anchor_position - odometry_.position
                            - rotation_matrix_ * tether.mounting_pos).normalized();
    }
}

void SlidingModeController::UpdateDynamicParams()
{

    CalculateRotationMatrix(odometry_.orientation, &rotation_matrix_);

    CalculateGlobalInertia(system_parameters_.inertia_, rotation_matrix_, &global_inertia_);

    CalculateAngularMappingMatrix(odometry_.orientation, &angular_mapping_matrix_);

    CalculateJacobian(system_parameters_.tether_configuration_, rotation_matrix_, &jacobian_);

    CalculateSpatialInertiaMatrix(system_parameters_, global_inertia_,
                                  angular_mapping_matrix_, &spatial_mass_matrix_);

    CalculateCentrifugalCoriolisMatrix(odometry_.angular_velocity, global_inertia_,
                                       angular_mapping_matrix_, derivative_angular_mapping_matrix_, &centrifugal_coriolis_matrix_);
}

void SlidingModeController::SetDesiredTrajectory(const pyramid_msgs::EigenMultiDOFJointTrajectory& trajectory)
{
    desired_trajectory_ = trajectory;
}

void SlidingModeController::SetFeedbackOdometry(const pyramid_msgs::EigenOdometry& odometry)
{
    odometry_ = odometry;
}

void SlidingModeController::CalculateSlidingSurface()
{
    CalculatePosAttDelta(odometry_, desired_trajectory_, &x_error_);
    CalculateVelocityDelta(odometry_, desired_trajectory_, &v_error_);

    sliding_surface_ = - v_error_ - system_parameters_.Lambda_ * x_error_;
}

void SlidingModeController::CalculateThrust()
{
    Eigen::VectorXd G(6);
    double mg = system_parameters_.mass_ * kDefaultGravity;
    G << 0.0, 0.0, mg, 0.0, 0.0, 0.0;

    Eigen::VectorXd acc_d = desired_trajectory_.getAcc();
    Eigen::VectorXd sgn_s = sgn(sliding_surface_);
    Eigen::VectorXd vel_odom = odometry_.getVel();

    wrench_
        = - spatial_mass_matrix_
          * (acc_d + system_parameters_.Lambda_ * v_error_ + system_parameters_.K_ * sgn_s)
          + centrifugal_coriolis_matrix_ * vel_odom + G;

    pyramid_msgs::vectorToEigenThrust(wrench_, &thrust_);
}

} //namespace system_commander
