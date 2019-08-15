#ifndef PYRAMID_MSGS_CONVERSIONS_H
#define PYRAMID_MSGS_CONVERSIONS_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include "pyramid_msgs/common.h"
#include "pyramid_msgs/pyramid_eigen_msgs.h"

namespace pyramid_msgs
{

inline void eigenThrustFromMsg(const geometry_msgs::WrenchStampedPtr& msg,
                                         pyramid_msgs::EigenWrenchStamped* thrust)
{
    assert(thrust != NULL);
    thrust->timestamp_ns = msg->header.stamp.toNSec();
    thrust->force_ET = vector3FromMsg(msg->wrench.force);
    thrust->torque_ET = vector3FromMsg(msg->wrench.torque);
}

} //namespace pyramid_msgs

#endif //PYRAMID_MSGS_CONVERSIONS_H
