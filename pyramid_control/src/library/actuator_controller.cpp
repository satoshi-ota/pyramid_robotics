#include "pyramid_control/actuator_controller.h"
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl

namespace pyramid_control
{

ActuatorController::ActuatorController(SystemParameters* system_parameters)
    :system_parameters_(system_parameters)
{
    lpp_.AddVariableSet(std::make_shared<ExVariables>());
    lpp_.AddCostSet(std::make_shared<ExCost>());
    lpp_.PrintCurrent();

    ipopt_.SetOption("linear_solver", "mumps");
    ipopt_.SetOption("jacobian_approximation", "exact");
    ipopt_.SetOption("print_level", 1);
}

ActuatorController::~ActuatorController(){ }

void ActuatorController::InitializeParameters()
{
    calculateAllocationMatrix(system_parameters_->rotor_configuration_, &allocation_matrix_);
}

void ActuatorController::wrenchDistribution(const Eigen::VectorXd& wrench)
{
    Eigen::MatrixXd MatA = -system_parameters_->jacobian_.transpose();
    Eigen::MatrixXd MatB = calcRotorMatrix(system_parameters_->rotMatrix_) * allocation_matrix_;
    Eigen::MatrixXd MatAB(MatA.rows(), MatA.cols()+MatB.cols());
    MatAB << MatA, MatB;

    Eigen::MatrixXd MatS;
    MatS.resize(6, 6);
    MatS.setZero();
    MatS.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
    MatS.bottomRightCorner(3, 3) = system_parameters_->toOmega_;

    MatAB = MatS.transpose() * MatAB;

    Eigen::MatrixXd distributionMatrix = MatAB.transpose() * (MatAB * MatAB.transpose()).inverse();

    distributedWrench_ = distributionMatrix * wrench;

    Eigen::FullPivLU<Eigen::MatrixXd> lu(MatAB);
    lu.setThreshold(1e-5);
    kernel_ = lu.kernel();
    rank_ = lu.rank();
}

void ActuatorController::optimize()
{
    if (feasibility() == false)
    {
        if(rank_ == 6)
        {
            lpp_.AddConstraintSet(std::make_shared<ExConstraint>("constraint1",
                                                                 kernel_, distributedWrench_));
            ipopt_.Solve(lpp_);
            Eigen::VectorXd x = lpp_.GetOptVariables()->GetValues();

            distributedWrench_ += kernel_ * x;

            tension_ = distributedWrench_.topLeftCorner(8, 1);
            tension_ = tension_.cwiseMax(Eigen::VectorXd::Zero(tension_.rows()));
            motor_speed_ = distributedWrench_.bottomLeftCorner(4, 1);
            motor_speed_ = motor_speed_.cwiseMax(Eigen::VectorXd::Zero(motor_speed_.rows()));
            motor_speed_ = motor_speed_.cwiseSqrt();

            lpp_.Clear();
        }
    }

}

} //namespace pyramid_control
