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
#include "pyramid_control/configurations.h"
#include "pyramid_control/tension_optimizer.h"

using namespace std;
using namespace ifopt;

namespace pyramid_control
{

class ActuatorController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ActuatorController(SystemParameters* system_parameters);
    ~ActuatorController();

    void InitializeParameters();

    void wrenchDistribution(const Eigen::VectorXd& wrench, const Eigen::VectorXd& disturbance);
    void optimize(Eigen::VectorXd* ref_tensions, Eigen::VectorXd* ref_rotor_velocities);

    inline bool feasibility(){return feasible_ = (distributedWrench_.array() >= 0).all();}

    SystemParameters *system_parameters_ = new SystemParameters();
    Problem lpp_;
    IpoptSolver ipopt_;

private:
    bool feasible_;
    unsigned int rank_;

    Eigen::Matrix4Xd allocation_matrix_;
    Eigen::MatrixXd kernel_;
    Eigen::VectorXd distributedWrench_;
};

} //pyramid_control

#endif //PYRAMID_CONTROL_ACTUATOR_CONTROLLER_H
