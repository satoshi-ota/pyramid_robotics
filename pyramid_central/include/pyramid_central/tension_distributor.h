#ifndef PYRAMID_CENTRAL_TENSION_DISTRIBUTOR_H
#define PYRAMID_CENTRAL_TENSION_DISTRIBUTOR_H

#include <Eigen/QR>
#include <iostream>

#include <ros/ros.h>

#include "pyramid_central/common.h"
#include "pyramid_central/configurations.h"

using namespace std;

namespace system_commander
{

class TensionDistributor
{
public:
    TensionDistributor();
    ~TensionDistributor();

    void TensionDistribution(const Eigen::VectorXd& wrench,
                             const Eigen::MatrixXd& jacobian,
                             const Eigen::Matrix3d& rotation_matrix,
                             const Eigen::Matrix3d& to_omega_matrix);
    void OptimizeTension();

    inline Eigen::Vector4d getTension(){return tension_;};
    inline Eigen::Vector4d getThrust(){return thrust_;};

    //from parameters.h
    SystemParameters system_parameters_;
private: //member data
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    bool feasible_tension_;

    Eigen::MatrixXd jacobian_kernel_;
    unsigned int jacobian_rank_;
    Eigen::MatrixXd jacobian_tilde_;

    Eigen::Vector4d tension_;
    Eigen::Vector4d thrust_;

    void CheckTensionFeasibility(){feasible_tension_ = (tension_.array() >= 0).all();}
};

} //system_commander

#endif //PYRAMID_CENTRAL_TENSION_DISTRIBUTOR_H
