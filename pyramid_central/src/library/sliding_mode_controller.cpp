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
     x_delta_(Eigen::VectorXd::Zero(6)),
     v_delta_(Eigen::VectorXd::Zero(6)),
     input_acceleration_(Eigen::VectorXd::Zero(6)){ }

SlidingModeController::~SlidingModeController(){ }

void SlidingModeController::UpdateDynamicParams()
{

    CalculateRotationMatrix(odometry_.orientation_EO, &rotation_matrix_);

    CalculateGlobalInertia(system_parameters_.inertia_, rotation_matrix_, &global_inertia_);

    CalculateAngularMappingMatrix(odometry_.orientation_EO, &angular_mapping_matrix_);

    CalculateSpatialInertiaMatrix(system_parameters_, global_inertia_,
                                  angular_mapping_matrix_, &spatial_mass_matrix_);

    CalculateCentrifugalCoriolisMatrix(odometry_.angular_velocity_EO, global_inertia_,
                                       angular_mapping_matrix_, derivative_angular_mapping_matrix_, &centrifugal_coriolis_matrix_);
}

void SlidingModeController::SetDesiredTrajectory(const EigenMultiDOFJointTrajectory& trajectory)
{
    desired_trajectory_ = trajectory;
    ROS_INFO("Set desired trajectory: position[%f, %f, %f].",
             desired_trajectory_.position_ET.x(),
             desired_trajectory_.position_ET.y(),
             desired_trajectory_.position_ET.z());
}

void SlidingModeController::SetFeedbackOdometry(const EigenOdometry& odometry)
{
    odometry_ = odometry;
}

void SlidingModeController::CalculateSlidingSurface()
{
    CalculatePosAttDelta(odometry_, desired_trajectory_, &x_delta_);
    CalculateVelocityDelta(odometry_, desired_trajectory_, &v_delta_);

    sliding_surface_ = v_delta_ + system_parameters_.Lambda_ * x_delta_;
}

void SlidingModeController::CalculateThrust()
{
    Eigen::VectorXd G = Eigen::VectorXd::Zero(6);
    G(2) = system_parameters_.mass_ * kDefaultGravity;

    Eigen::VectorXd desired_acc = desired_trajectory_.getAcc();

    Eigen::VectorXd sgn_s = sgn(sliding_surface_);
    Eigen::VectorXd odom_vel = odometry_.getVel();

    wrench_
        = - spatial_mass_matrix_
          * (desired_acc - system_parameters_.Lambda_ * v_delta_ + system_parameters_.K_ * sgn_s)
          + centrifugal_coriolis_matrix_ * odom_vel + G;

    thrust_.force = wrench_.block<3, 1>(0, 0);
    thrust_.torque = wrench_.block<3, 1>(3, 0);
}


} //namespace system_commander
