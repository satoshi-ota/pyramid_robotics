#include "pyramid_control/observer.h"

namespace pyramid_control
{

Observer::Observer(SystemParameters* system_parameters)
    :system_parameters_(system_parameters),
     torqueOBSGain_(Eigen::Vector3d(0.1, 0.1, 0.1).asDiagonal()),
     beta_(Eigen::Vector3d::Zero())
{
    const double kDt = 0.1;

    disturbance_.resize(6, 1);
    disturbance_.setZero();

    xEst_.resize(9, 1);
    xEst_.setZero();

    PEst_.resize(9, 9);
    PEst_ = Eigen::MatrixXd::Identity(9, 9);

    kA_.resize(9, 9);
    kA_ = Eigen::MatrixXd::Identity(9, 9);
    kA_.block(0, 3, 3, 3) = Eigen::Vector3d(kDt, kDt, kDt).asDiagonal();
    kA_.block(3, 6, 3, 3) = Eigen::Vector3d(kDt, kDt, kDt).asDiagonal();

    kBu_.resize(9, 3);
    kBu_.setZero();
    kBu_.block(3, 0, 3, 3) = Eigen::Vector3d(kDt, kDt, kDt).asDiagonal();

    kB_.resize(9, 9);
    kB_.setZero();
    kB_ = Eigen::MatrixXd::Identity(9, 9);
    kB_ = kB_.array() * kDt;

    kC_.resize(3, 9);
    kC_.setZero();
    kC_.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

    kG_.resize(9, 1);
    kG_ << 0.0, 0.0, 0.0, 0.0, 0.0, -kDefaultGravity*kDt, 0.0, 0.0, 0.0;

    kQ_.resize(9, 9);
    kQ_ = Eigen::MatrixXd::Identity(9, 9);

    kR_.resize(3, 3);
    kR_ = Eigen::Matrix3d::Identity();
}

Observer::~Observer(){ }

void Observer::estimateDisturbance(const Eigen::VectorXd& wrench,
                                         Eigen::VectorXd* disturbance)
{
    assert(disturbance != NULL);

    Eigen::VectorXd u = wrench - disturbance_;

    fEst(u);
    tEst(u);

    *disturbance = disturbance_;
}

void Observer::fEst(const Eigen::VectorXd& u)
{
    Eigen::VectorXd xPred = kA_ * xEst_ + kBu_ * u.topLeftCorner(3, 1) + kG_;
    Eigen::MatrixXd PPred = kA_ * PEst_ * kA_.transpose() + kB_ * kQ_ * kB_.transpose();

    Eigen::MatrixXd kalmanGain
        = PPred * kC_.transpose() * (kC_ * PPred * kC_.transpose() + kR_).inverse();

    xEst_ = xPred + kalmanGain * (system_parameters_->odometry_.position - kC_ * xPred);
    PEst_ = (Eigen::MatrixXd::Identity(9, 9) - kalmanGain * kC_) * PPred;

    disturbance_.topLeftCorner(3, 1) = xEst_.bottomLeftCorner(3, 1);
}

void Observer::tEst(const Eigen::VectorXd& u)
{
    beta_ += torqueOBSGain_ * (system_parameters_->omegaXmomentum_ - u.bottomLeftCorner(3, 1) - disturbance_.bottomLeftCorner(3, 1));

    disturbance_.bottomLeftCorner(3, 1) = beta_ + torqueOBSGain_ * system_parameters_->momentum_;
}

} //namespace pyramid_control
