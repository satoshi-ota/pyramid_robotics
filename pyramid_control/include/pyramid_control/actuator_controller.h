#ifndef PYRAMID_CONTROL_ACTUATOR_CONTROLLER_H
#define PYRAMID_CONTROL_ACTUATOR_CONTROLLER_H

#include <Eigen/QR>
#include <iostream>
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include <ros/ros.h>

#include "pyramid_control/common.h"
#include "pyramid_control/common_central.h"
#include "pyramid_control/configurations.h"
#include "pyramid_control/tension_optimizer.h"

using namespace std;
using namespace ifopt;

namespace pyramid_control
{

class ActuatorController
{
public:
    ActuatorController();
    ~ActuatorController();

    void InitializeParameters();

    void wrenchDistribution(const Eigen::VectorXd& wrench,
                            const Eigen::MatrixXd& jacobian,
                            const Eigen::Matrix3d& rotMatrix,
                            const Eigen::Matrix3d& toOmega);
    void optimize();

    inline Eigen::Vector4d getTension(){return tension_;};
    inline Eigen::Vector4d getThrust(){return thrust_;};
    inline bool feasibility(){return feasible_ = (distributedWrench_.array() >= 0).all();}

    SystemParameters system_parameters_;
    Problem lpp_;
    IpoptSolver ipopt_;

private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix4Xd allocation_matrix_;

    bool feasible_;
    unsigned int rank_;

    Eigen::MatrixXd kernel_;
    Eigen::MatrixXd jacobian_tilde_;

    Eigen::VectorXd distributedWrench_;

    Eigen::VectorXd tension_;
    Eigen::VectorXd thrust_;


};

} //pyramid_control

#endif //PYRAMID_CONTROL_ACTUATOR_CONTROLLER_H
