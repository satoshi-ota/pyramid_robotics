#include "pyramid_control/sliding_mode_controller.h"
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl

namespace pyramid_control
{

SlidingModeController::SlidingModeController()
    :rotMatrix_(Eigen::Matrix3d::Zero()),
     globalInertia_(Eigen::Matrix3d::Zero()),
     toOmega_(Eigen::Matrix3d::Zero()),
     toOmega_dot_(Eigen::Matrix3d::Zero()),
     massMatrix_(Eigen::MatrixXd::Zero(6, 6)),
     coriolisMatrix_(Eigen::MatrixXd::Zero(6, 6)),
     wrench_(Eigen::VectorXd::Zero(6)),
     slidingSurface_(Eigen::VectorXd::Zero(6)),
     xError_(Eigen::VectorXd::Zero(6)),
     vError_(Eigen::VectorXd::Zero(6)){ }

SlidingModeController::~SlidingModeController(){ }

void SlidingModeController::updateModelConfig()
{
    direction.clear();

    int i = 0;
    for(PseudoTether& pseudo_tether : system_parameters_.tether_configuration_.pseudo_tethers)
    {
        pseudo_tether.update(odometry_);
        direction.push_back(pseudo_tether.direction);
    }

    rotMatrix_ = odometry_.orientation.toRotationMatrix();
    globalInertia_ = rotMatrix_ * system_parameters_.inertia_ * rotMatrix_.transpose();

    calcToOmage(odometry_.orientation, &toOmega_);

    calcJacobian(system_parameters_.tether_configuration_, rotMatrix_, &jacobian_);

    calcMassMatrix(system_parameters_, globalInertia_, toOmega_, &massMatrix_);

    calcCoriolisMatrix(odometry_.angular_velocity, globalInertia_,
                                       toOmega_, toOmega_dot_, &coriolisMatrix_);
}

void SlidingModeController::calcThrust()
{
    calcPosAttDelta(odometry_, trajectory_, &xError_);
    calcVelDelta(odometry_, trajectory_, &vError_);

    slidingSurface_ = - vError_ - system_parameters_.Lambda_ * xError_;

    Eigen::VectorXd kG(6);
    double mg = system_parameters_.mass_ * kDefaultGravity;
    kG << 0.0, 0.0, mg, 0.0, 0.0, 0.0;

    Eigen::VectorXd sgn_s = sgn(slidingSurface_);
    Eigen::VectorXd vel = odometry_.getVel();
    Eigen::VectorXd acc = trajectory_.getAcc();

    wrench_ = - massMatrix_
            * (acc + system_parameters_.Lambda_ * vError_ + system_parameters_.K_ * sgn_s)
            + coriolisMatrix_ * vel + kG;

    pyramid_msgs::vectorToEigenThrust(wrench_, &thrust_);
}

} //namespace pyramid_control
