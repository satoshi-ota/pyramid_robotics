#ifndef SIMPLE_TENSIONS_CONTROLLER_H
#define SIMPLE_TENSIONS_CONTROLLER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

namespace tensions_control
{

class SimpleTensionsController
{
public:
    SimpleTensionsController();
    ~SimpleTensionsController();

    void MsgFromEigenTensions();

protected:

};

} //namespace tensions_control

#endif //SIMPLE_TENSION_CONTROLLER_H
