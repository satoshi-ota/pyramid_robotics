#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <pyramid_central/tension_optimizer.h>

using namespace ifopt;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "system_commander_node");

    Eigen::MatrixXd kernel = Eigen::MatrixXd::Zero(4, 2);
    Eigen::Vector4d tau = Eigen::Vector4d::Zero();

    kernel <<        1,         0,
                0.260452,  0.970258,
                    0.944191, -0.334938,
                    0,         1;

    tau << -0.650899,-0.632199, 0.412625, 0.437665;

    // 1. define the problem
    Problem nlp;
    nlp.AddVariableSet  (std::make_shared<ExVariables>());
    nlp.AddConstraintSet(std::make_shared<ExConstraint>("constraint1", kernel, tau));
    nlp.AddCostSet      (std::make_shared<ExCost>());
    nlp.PrintCurrent();

    // 2. choose solver and options
    IpoptSolver ipopt;
    ipopt.SetOption("linear_solver", "mumps");
    ipopt.SetOption("jacobian_approximation", "exact");

    // 3 . solve
    ipopt.Solve(nlp);
    Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
    //std::cout << x.transpose() << std::endl;

    std::cout << (tau + kernel * x).transpose() << std::endl;

    // 4. test if solution correct
    //double eps = 1e-5; //double precision
    //assert(2.0-eps < x(0) && x(0) < 2.0+eps);
    //assert(4.0-eps < x(1) && x(1) < 4.0+eps);

    ros::spin();

    return 0;
}
