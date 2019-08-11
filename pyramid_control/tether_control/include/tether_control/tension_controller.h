#ifndef TENSION_CONTROLLER_H
#define TENSION_CONTROLLER_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LinkState.h>
#include <geometry_msgs/Pose.h>

class TensionController
{
public:

protected:
    //subscriber to tether tensions
    ros::Subscriber tensions_command_;
    sensor_msgs::JointState joint_state_;


};
