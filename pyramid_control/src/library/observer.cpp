#include "pyramid_control/observer.h"
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl

namespace pyramid_control
{

Observer::Observer(SystemParameters* system_parameters)
    :system_parameters_(system_parameters),
     forceDisturbance_(Eigen::Vector3d::Zero()),
     torqueDisturbance_(Eigen::Vector3d::Zero()),
     torqueOBSGain_(Eigen::Matrix3d::Identity())
{
    xEst_.resize(9, 1);
    xEst_.setZero();

    PEst_.resize(9, 9);
    PEst_ = Eigen::MatrixXd::Identity(9, 9);

    matA_.resize(9, 9);
    matA_.setZero();
    matA_ = Eigen::MatrixXd::Identity(9, 9);
    matA_.block(0, 3, 3, 3) = Eigen::Vector3d(kdt, kdt, kdt).asDiagonal();
    matA_.block(3, 6, 3, 3) = Eigen::Vector3d(kdt, kdt, kdt).asDiagonal();

    matB_.resize(9, 3);
    matB_.setZero();
    matB_.block(3, 0, 3, 3) = Eigen::Vector3d(kdt, kdt, kdt).asDiagonal();

    matC_.resize(3, 9);
    matC_.setZero();
    matC_.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();


    covQ << 0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1;

    covR << 0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1;
}

Observer::~Observer(){ }

void Observer::estimateDisturbance(const Eigen::VectorXd& wrench)
{
    Eigen::VectorXd x;
    Eigen::MatrixXd cov;

    Eigen::VectorXd kG(9);
    kG << 0.0, 0.0, 0.0, 0.0, 0.0, -kDefaultGravity*kdt, 0.0, 0.0, 0.0;

    x = matA_ * xEst_ + matB_ * wrench.topLeftCorner(3, 1) + kG;
    cov = matA_ * PEst_ * matA_.transpose() + matB_ * covQ * matB_.transpose();

    Eigen::MatrixXd kalmanGain;

    kalmanGain = cov * matC_.transpose() * (matC_ * cov * matC_.transpose() + covR).inverse();
    xEst_ = x + kalmanGain * (system_parameters_->odometry_.position - matC_ * x);
    PEst_ = (Eigen::MatrixXd::Identity(9, 9) - kalmanGain * matC_) * cov;

    // PRINT_MAT(matA_);
    // PRINT_MAT(matB_);
    // PRINT_MAT(matC_);
    PRINT_MAT(xEst_);

    beta_ += torqueOBSGain_ * (wrench.bottomLeftCorner(3, 1) - system_parameters_->skewMatrix_ * system_parameters_->inertia_ * system_parameters_->odometry_.angular_velocity - torqueDisturbance_);

    beta_ = beta_.array() * kdt;

    torqueDisturbance_ = beta_ + torqueOBSGain_ * system_parameters_->inertia_ * system_parameters_->odometry_.angular_velocity;

    // PRINT_MAT(torqueDisturbance_);
}

} //namespace pyramid_control
