#ifndef MOTOR_SPEED_CONTROLLER_CONVERSIONS_H
#define MOTOR_SPEED_CONTROLLER_CONVERSIONS_H

namespace conversions
{

inline void eigenThrustFromMsg(const geometry_msgs::WrenchStamped& msg,EigenWrenchStamped* thrust)
{
    assert(thrust != NULL);
    thrust->timestamp_ns = msg.header.stamp.toNSec();
    thrust->force_ET = vector3FromMsg(msg.force);
    thrust->torque_ET = vector3FromMsg(msg.torque);
}

inline Eigen::Vector3d vector3FromMsg(const geometry_msgs::Vector3& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline void calculateAllocationMatrix(const RotorConfiguration& rotor_configuration,
                                        Eigen::Matrix4Xd* allocation_matrix)
{
    assert(allocation_matrix != nullptr);
    allocation_matrix->resize(4, rotor_configuration.rotors.size());
    unsigned int i = 0;
    for (const Rotor& rotor : rotor_configuration.rotors) {
        // Set first row of allocation matrix.
        (*allocation_matrix)(0, i) = sin(rotor.angle) * rotor.arm_length
                                        * rotor.rotor_force_constant;
        // Set second row of allocation matrix.
        (*allocation_matrix)(1, i) = -cos(rotor.angle) * rotor.arm_length
                                        * rotor.rotor_force_constant;
        // Set third row of allocation matrix.
        (*allocation_matrix)(2, i) = -rotor.direction * rotor.rotor_force_constant
                                        * rotor.rotor_moment_constant;
        // Set forth row of allocation matrix.
        (*allocation_matrix)(3, i) = rotor.rotor_force_constant;
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

} //namespace conversions

#endif //MOTOR_SPEED_CONTROLLER_CONVERSIONS_H
