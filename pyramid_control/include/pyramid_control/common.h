#ifndef PYRAMID_CONTROL_COMMON_H
#define PYRAMID_CONTROL_COMMON_H

#include <assert.h>

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

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_COMMON_H
