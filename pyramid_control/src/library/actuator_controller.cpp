#include "pyramid_control/actuator_controller.h"

namespace pyramid_control
{

ActuatorController::ActuatorController(SystemParameters* system_parameters)
    :system_parameters_(system_parameters)
{
    Eigen::VectorXd initVal;
    initVal.resize(6, 1);
    initVal.setZero();

    lpp_.AddVariableSet(std::make_shared<ExVariables>("var_set1", initVal, 6));
    lpp_.AddCostSet(std::make_shared<ExCost>());
    lpp_.PrintCurrent();

    ipopt_.SetOption("linear_solver", "mumps");
    ipopt_.SetOption("jacobian_approximation", "exact");
    ipopt_.SetOption("print_level", 1);

    upperLimmit_.resize(12, 1);
}

ActuatorController::~ActuatorController(){ }

void ActuatorController::InitializeParameters()
{
    calculateAllocationMatrix(system_parameters_->rotor_configuration_, &allocation_matrix_);
}

void ActuatorController::wrenchDistribution(const Eigen::VectorXd& wrench,
                                            const Eigen::VectorXd& disturbance)
{
    Eigen::MatrixXd MatA = system_parameters_->jacobian_.transpose();
    Eigen::MatrixXd MatB = calcRotorMatrix(system_parameters_->rotMatrix_) * allocation_matrix_;
    Eigen::MatrixXd MatAB(MatA.rows(), MatA.cols()+MatB.cols());
    MatAB << MatA, MatB;

    Eigen::MatrixXd distributionMatrix = MatAB.transpose() * (MatAB * MatAB.transpose()).inverse();

    distributedWrench_ = distributionMatrix * (wrench - disturbance);

    Eigen::FullPivLU<Eigen::MatrixXd> lu(MatAB);
    lu.setThreshold(1e-5);
    kernel_ = lu.kernel();
    rank_ = lu.rank();
}

void ActuatorController::optimize(Eigen::VectorXd* ref_tensions, Eigen::VectorXd* ref_rotor_velocities)
{
    if (feasibility() == false)
    {
        if(rank_ == 6)
        {
            lpp_.AddConstraintSet(std::make_shared<ExConstraint>("constraint1", 12,
                                                                 kernel_,
                                                                 distributedWrench_,
                                                                 upperLimmit_));
            ipopt_.Solve(lpp_);
            Eigen::VectorXd x = lpp_.GetOptVariables()->GetValues();

            distributedWrench_ += kernel_ * x;

            *ref_tensions = distributedWrench_.topLeftCorner(8, 1);
            *ref_tensions = ref_tensions->cwiseMax(Eigen::VectorXd::Zero(ref_tensions->rows()));
            *ref_rotor_velocities = distributedWrench_.bottomLeftCorner(4, 1);
            *ref_rotor_velocities
                = ref_rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(ref_rotor_velocities->rows()));
            *ref_rotor_velocities = ref_rotor_velocities->cwiseSqrt();

            lpp_.Clear();
        }
    }

}

} //namespace pyramid_control
