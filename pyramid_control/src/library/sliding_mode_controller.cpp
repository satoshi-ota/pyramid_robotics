#include "pyramid_control/sliding_mode_controller.h"

namespace pyramid_control
{

SlidingModeController::SlidingModeController(SystemParameters* system_parameters)
    :system_parameters_(system_parameters),
     wrench_(Eigen::VectorXd::Zero(6)),
     slidingSurface_(Eigen::VectorXd::Zero(6)),
     xError_(Eigen::VectorXd::Zero(6)),
     vError_(Eigen::VectorXd::Zero(6)){ }

SlidingModeController::~SlidingModeController(){ }

void SlidingModeController::updateModelConfig()
{
    system_parameters_->update();
    system_parameters_->calcskewMatrix();
    system_parameters_->calcToOmage();
    system_parameters_->calcMassMatrix();
    system_parameters_->calcCoriolisMatrix();
    system_parameters_->calcJacobian();
}

void SlidingModeController::calcThrust()
{
    calcPosAttDelta(system_parameters_->odometry_, trajectory_, &xError_);
    calcVelDelta(system_parameters_->odometry_, trajectory_, &vError_);

    slidingSurface_ = - vError_ - system_parameters_->Lambda_ * xError_;

    Eigen::VectorXd kG(6);
    double mg = system_parameters_->mass_ * kDefaultGravity;
    kG << 0.0, 0.0, mg, 0.0, 0.0, 0.0;

    Eigen::VectorXd sgn_s = sgn(slidingSurface_);
    Eigen::VectorXd vel = system_parameters_->odometry_.getVel();
    Eigen::VectorXd acc = trajectory_.getAcc();

    wrench_ = - system_parameters_->massMatrix_
            * (acc + system_parameters_->Lambda_ * vError_ + system_parameters_->K_ * sgn_s)
            + system_parameters_->coriolisMatrix_ * vel + kG;

    pyramid_msgs::vectorToEigenThrust(wrench_, &thrust_);
}

} //namespace pyramid_control
