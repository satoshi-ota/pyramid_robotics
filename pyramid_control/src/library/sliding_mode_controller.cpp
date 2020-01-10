#include "pyramid_control/sliding_mode_controller.h"

namespace pyramid_control
{

SlidingModeController::SlidingModeController(SystemParameters* system_parameters)
    :system_parameters_(system_parameters){}

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

void SlidingModeController::calcThrust(Eigen::VectorXd* wrench)
{
    assert(wrench != NULL);

    Eigen::VectorXd xError;
    Eigen::VectorXd vError;

    calcPosAttDelta(system_parameters_->odometry_, trajectory_, &xError);
    calcVelDelta(system_parameters_->odometry_, trajectory_, &vError);

    Eigen::VectorXd slidingSurface = - vError - system_parameters_->Lambda_ * xError;

    Eigen::VectorXd kG(6);
    double mg = system_parameters_->mass_ * kDefaultGravity;
    kG << 0.0, 0.0, mg, 0.0, 0.0, 0.0;

    Eigen::VectorXd sgn_s = sgn(slidingSurface);
    Eigen::VectorXd vel = system_parameters_->odometry_.getVel();
    Eigen::VectorXd acc = trajectory_.getAcc();

    *wrench = - system_parameters_->massMatrix_
            * (acc + system_parameters_->Lambda_ * vError + system_parameters_->K_ * sgn_s)
            + system_parameters_->coriolisMatrix_ * vel + kG;
}

} //namespace pyramid_control
