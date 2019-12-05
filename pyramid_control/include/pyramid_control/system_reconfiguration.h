#ifndef PYRAMID_CENTRAL_SYSTEM_RECONFIGURATION_H
#define PYRAMID_CENTRAL_SYSTEM_RECONFIGURATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "pyramid_control/SlidingModeControllerConfig.h"
#include "pyramid_control/TrajectoryGeneratorConfig.h"
#include "pyramid_control/parameters.h"

namespace pyramid_control
{

class SystemReconfigure
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SystemReconfigure()
        :pos_(Eigen::Vector3d::Zero()),
         att_(Eigen::Vector3d::Zero()){ }

    ~SystemReconfigure(){ }

    void smcReconfig(pyramid_control::SlidingModeControllerConfig& cfg,
                                       SystemParameters* system_parameters);

    void trajectoryReconfig(pyramid_control::TrajectoryGeneratorConfig& cfg);

    inline Eigen::Vector3d getDesiredPos(){return pos_;};
    inline Eigen::Vector3d getDesiredAtt(){return att_;};

    // inline double getRoll(){return roll_;};
    // inline double getPitch(){return pitch_;};
    // inline double getYaw(){return yaw_;};

private:
    //general
    Eigen::Vector3d pos_;
    Eigen::Vector3d att_;
    // double roll_;
    // double pitch_;
    // double yaw_;
};

} //namespace pyramid_control

#endif //PYRAMID_CENTRAL_SYSTEM_RECONFIGURATION_H
