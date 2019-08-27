#include "pyramid_central/tension_distributor.h"
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl

namespace system_commander
{

TensionDistributor::TensionDistributor()
{
    lpp_.AddVariableSet(std::make_shared<ExVariables>());
    lpp_.AddCostSet(std::make_shared<ExCost>());

    ipopt_.SetOption("linear_solver", "mumps");
    ipopt_.SetOption("jacobian_approximation", "exact");
}

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

    tension_thrust_ = H.transpose() * (H * H.transpose()).inverse() * wrench;

    Eigen::FullPivLU<Eigen::MatrixXd> lu(H);
    lu.setThreshold(1e-5);
    kernel_ = lu.kernel();
    rank_ = lu.rank();
}

void TensionDistributor::OptimizeTension()
{
    CheckTensionFeasibility();

    if (feasible_ == false)
    {
        if (rank_ == 6)
        {
            //optimize
            lpp_.AddConstraintSet(std::make_shared<ExConstraint>("constraint1",
                                                                 kernel_.block<4, 2>(0, 0), tension_thrust_.block<4, 1>(0, 0)));
            ipopt_.Solve(lpp_);
            Eigen::VectorXd x = lpp_.GetOptVariables()->GetValues();

            tension_thrust_ += kernel_ * x;
            tension_ = tension_thrust_.block<4, 1>(0, 0);
            thrust_ = tension_thrust_.block<4, 1>(4, 0);

            lpp_.Clear();
        }
    }
    else
    {
        tension_ = tension_thrust_.block<4, 1>(0, 0);
        thrust_ = tension_thrust_.block<4, 1>(4, 0);
    }
}

} //namespace
