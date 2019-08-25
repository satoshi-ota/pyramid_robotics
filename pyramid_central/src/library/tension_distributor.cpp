#include "pyramid_central/tension_distributor.h"
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl

namespace system_commander
{

TensionDistributor::TensionDistributor(){ }

TensionDistributor::~TensionDistributor(){ }

void TensionDistributor::TensionDistribution(const Eigen::VectorXd& wrench,
                                             const Eigen::MatrixXd& jacobian,
                                             const Eigen::Matrix3d& rotation_matrix,
                                             const Eigen::Matrix3d& to_omega_matrix)
{
    Eigen::Matrix<double, 6, 6> S = Eigen::MatrixXd::Zero(6, 6);
    S.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    S.block<3, 3>(3, 3) = to_omega_matrix;

    jacobian_tilde_ = jacobian * S;

    Eigen::Matrix<double, 6, 8> H = Eigen::MatrixXd::Zero(6, 8);
    H.block<6, 4>(0, 0) = jacobian_tilde_.transpose();
    H(0, 4) = rotation_matrix(0, 2);
    H(1, 4) = rotation_matrix(1, 2);
    H(2, 4) = rotation_matrix(2, 2);
    H.block<3, 3>(3, 5) = Eigen::Matrix3d::Identity();

    Eigen::VectorXd tension_thrust = H.transpose() * (H * H.transpose()).inverse() * wrench;

    tension_ = tension_thrust.block<4, 1>(0, 0);
    thrust_ = tension_thrust.block<4, 1>(4, 0);
}

void TensionDistributor::OptimizeTension()
{
    CheckTensionFeasibility();
    //PRINT_MAT(jacobian_tilde_.transpose());

    if (feasible_tension_ == false)
    {
        Eigen::FullPivLU<Eigen::MatrixXd> lu(jacobian_tilde_.transpose());
        lu.setThreshold(1e-1);
        jacobian_kernel_ = lu.kernel();
        jacobian_rank_ = lu.rank();

        if (jacobian_rank_ == 4)
            ROS_WARN("Coundn't find feasible tension.");
        else
            
        //cout << "The rank of J is " << lu.rank() << endl;
        PRINT_MAT(jacobian_kernel_);
    }
}

} //namespace
