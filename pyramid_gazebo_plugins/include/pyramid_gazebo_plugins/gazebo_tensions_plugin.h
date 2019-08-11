#ifndef PYRAMID_GAZEBO_PLUGINS_GAZEBO_TENSIONS_PLUGIN_H
#define PYRAMID_GAZEBO_PLUGINS_GAZEBO_TENSIONS_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkState.h>
#include <pyramid_msgs/Tensions.h>

namespace gazebo{

class TensionsPlugin : public ModelPlugin
{
public:
    TensionsPlugin();
    ~TensionsPlugin();

    virtual void Load();
    virtual void Update();

private: //member function

    //tensions distribution
    void TensionsCommandCB(const pyramid_msgs::TensionsConsrPtr &msg);

private: //data member
    //subscriber to tensions
    ros::Subscriber command_subscriber_;
    sensor_msgs::JointState joint_command_;

    //publisher to joint_state
    ros::Publisher joint_state_publisher_;
    sensor_msgs::JointState joint_states_;

    //publisher to end-effector position
    ros::Publisher ee_publisher_;
    
};

} //namespace gazebo

#endif //PYRAMID_GAZEBO_PLUGINS_GAZEBO_TENSIONS_PLUGIN_H
