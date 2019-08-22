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
    SystemReconfigure()
        :desired_position_(Eigen::Vector3d::Zero()),
         desired_yaw_(0.0){ }

    ~SystemReconfigure(){ }

    void ControllerReconfig(pyramid_central::SystemCommanderConfig& cfg, SystemParameters* system_parameters);
    void TrajectoryReconfig(pyramid_central::SystemCommanderConfig& cfg);

    inline Eigen::Vector3d getDesiredPosition(){return desired_position_;};
    inline double getDesiredYaw(){return desired_yaw_;};

private:
    //general
    Eigen::Vector3d desired_position_;
    double desired_yaw_;
};

} //namespace system_commander

#endif //PYRAMID_CENTRAL_SYSTEM_RECONFIGURATION_H
