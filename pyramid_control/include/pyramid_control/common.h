#ifndef PYRAMID_CONTROL_COMMON_H
#define PYRAMID_CONTROL_COMMON_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <assert.h>

#include <pyramid_msgs/common.h>
#include <pyramid_msgs/pyramid_eigen_msgs.h>
#include <pyramid_msgs/conversions.h>

#include "pyramid_control/parameters.h"

namespace pyramid_control
{

struct EigenWrenchStamped
{
    EigenWrenchStamped()
        :timestamp_ns(-1),
         force_ET(Eigen::Vector3d::Zero()),
         torque_ET(Eigen::Vector3d::Zero()){ }

    EigenWrenchStamped(const Eigen::Vector3d& _force, const Eigen::Vector3d& _torque)
        :force_ET(_force),
         torque_ET(_torque){ }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int64_t timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
    Eigen::Vector3d force_ET;
    Eigen::Vector3d torque_ET;

    inline double getZforce() const {return force_ET.z();}
    inline Eigen::Vector3d getTorque() const {return torque_ET;}
};

inline void eigenThrustFromMsg(const geometry_msgs::WrenchStampedPtr& msg, EigenWrenchStamped* thrust)
{
    assert(thrust != NULL);
    thrust->timestamp_ns = msg->header.stamp.toNSec();
    thrust->force_ET = pyramid_msgs::vector3FromMsg(msg->wrench.force);
    thrust->torque_ET = pyramid_msgs::vector3FromMsg(msg->wrench.torque);
}

inline void calculateAllocationMatrix(const RotorConfiguration& rotor_configuration,
                                            Eigen::Matrix4Xd* allocation_matrix)
{
    assert(allocation_matrix != nullptr);
    allocation_matrix->resize(4, rotor_configuration.rotors.size());
    unsigned int i = 0;
    for(const Rotor& rotor : rotor_configuration.rotors)
    {
        (*allocation_matrix)(0, i) = rotor.rotor_force_constant;
        (*allocation_matrix)(1, i) = sin(rotor.angle) * rotor.arm_length
            * rotor.rotor_force_constant;
        (*allocation_matrix)(2, i) = -cos(rotor.angle) * rotor.arm_length
            * rotor.rotor_force_constant;
        (*allocation_matrix)(3, i) = -rotor.direction * rotor.rotor_force_constant
            * rotor.rotor_moment_constant;
        ++i;
    }
    Eigen::FullPivLU<Eigen::Matrix4Xd> lu(*allocation_matrix);
    // Setting the threshold for when pivots of the rank calculation should be considered nonzero.
    lu.setThreshold(1e-9);
    int rank = lu.rank();
    if (rank < 4)
    {
        std::cout   << "The rank of the allocation matrix is " << lu.rank()
                    << ", it should have rank 4, to have a fully controllable system,"
                    << " check your configuration." << std::endl;
    }
}

inline void calcPosAttDelta(const pyramid_msgs::EigenOdometry& odometry,
                            const pyramid_msgs::EigenMultiDOFJointTrajectory& trajectory,
                                         Eigen::VectorXd* xError)
{
    *xError = trajectory.getPosAtt() - odometry.getPosAtt();
}

inline void calcVelDelta(const pyramid_msgs::EigenOdometry& odometry,
                         const pyramid_msgs::EigenMultiDOFJointTrajectory& trajectory,
                                      Eigen::VectorXd* vError)
{
    *vError = trajectory.getVel() - odometry.getGrobalVel();
}

inline void LimitTensions(Eigen::Vector4d* tensions)
{
    Eigen::Vector4d max_tensions(10.0, 10.0, 10.0, 10.0);
    *tensions = tensions->cwiseMax(Eigen::VectorXd::Zero(tensions->rows()));
    *tensions = tensions->cwiseMin(max_tensions);
}

inline Eigen::VectorXd sgn(Eigen::VectorXd& sliding_surface)
{
    Eigen::VectorXd sgn_s = Eigen::VectorXd::Zero(6);

    double alpha = 10;
    for (unsigned int i=0;i<sliding_surface.size();++i)
        sgn_s(i) = tanh(alpha * sliding_surface(i));

    return sgn_s;
}

inline Eigen::MatrixXd calcRotorMatrix(const Eigen::Matrix3d& rotMatrix)
{
    double rot13 = rotMatrix(0, 2);
    double rot23 = rotMatrix(1, 2);
    double rot33 = rotMatrix(2, 2);

    Eigen::MatrixXd Mat;
    Mat.resize(6, 4);
    Mat.setZero();

    Mat << rot13, 0, 0, 0,
           rot23, 0, 0, 0,
           rot33, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;

    return Mat;
}

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_COMMON_H
