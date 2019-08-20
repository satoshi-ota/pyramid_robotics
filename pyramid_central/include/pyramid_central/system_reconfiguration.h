#ifndef PYRAMID_CENTRAL_SYSTEM_RECONFIGURATION_H
#define PYRAMID_CENTRAL_SYSTEM_RECONFIGURATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "pyramid_central/SystemCommanderConfig.h"
#include "pyramid_central/parameters.h"

namespace system_commander
{

class SystemReconfigure
{
public:
    SystemReconfigure(){ }
    ~SystemReconfigure(){ }

    void reconfig(pyramid_central::SystemCommanderConfig& cfg, SystemParameters* system_parameters);
};

} //namespace system_commander

#endif //PYRAMID_CENTRAL_SYSTEM_RECONFIGURATION_H
