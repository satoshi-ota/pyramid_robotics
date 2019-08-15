#ifndef PYRAMID_MSGS_PYRAMID_EIGEN_MSGS_H
#define PYRAMID_MSGS_PYRAMID_EIGEN_MSGS_H

#include <Eigen/Eigen>

namespace pyramid_msgs
{

struct EigenWrenchStamped
{
    EigenWrenchStamped()
        :timestamp_ns(-1),
         force_ET(Eigen::Vector3d::Zero()),
         torque_ET(Eigen::Vector3d::Zero()){ }

    EigenWrenchStamped(const Eigen::Vector3d& _force,
                const Eigen::Vector3d& _torque)
        :force_ET(_force),
         torque_ET(_torque){ }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int64_t timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
    Eigen::Vector3d force_ET;
    Eigen::Vector3d torque_ET;
};

} //namespace pyramid_msgs

#endif //PYRAMID_MSGS_PYRAMID_EIGEN_MSGS_H
