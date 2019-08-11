#ifndef SIMPLE_TENSION_CONTROLLER_H
#define SIMPLE_TENSION_CONTROLLER_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LinkState.h>
#include <geometry_msgs/Pose.h>
#include <pyramid_msgs/Tension.h>

class SimpleTensionController
{
public:
    SimpleTensionController();
    ~SimpleTensionController();

protected:
    //publisher to tether tensions
    ros::Subscriber tensions_pub;
    sensor_msgs::JointState tensions_msg;

    //tensions distribution
    void TensionCommandCB(const pyramid_msgs::TensionConstPtr &msg);
};
