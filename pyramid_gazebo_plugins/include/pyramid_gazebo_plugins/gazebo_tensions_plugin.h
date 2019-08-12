#ifndef PYRAMID_GAZEBO_PLUGINS_GAZEBO_TENSIONS_PLUGIN_H
#define PYRAMID_GAZEBO_PLUGINS_GAZEBO_TENSIONS_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkState.h>

namespace gazebo{

class TensionsPlugin : public ModelPlugin
{
public:
    TensionsPlugin();
    ~TensionsPlugin()
    {
        rosnode_.shutdown();
    }

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Update();

private: //member function

    //tensions distribution
    void TensionsCommandCB(const sensor_msgs::JointStateConstPtr &_msg)
    {
        tensions_command_ = *_msg;
        command_received_ = true;
    }

private: //data member
    //general
    ros::NodeHandle rosnode_;
    ros::CallbackQueue callback_queue_;
    event::ConnectionPtr update_event_;
    double update_T_;
    double t_prev_;

    //model
    physics::ModelPtr model_;

    //joints
    std::vector<physics::JointPtr> joints_;
    double f_max;

    //subscriber to tensions
    bool use_tether_;
    ros::Subscriber command_subscriber_;
    sensor_msgs::JointState tensions_command_;
    bool command_received_;

    //publisher to joint_states
    ros::Publisher joint_state_publisher_;
    sensor_msgs::JointState joint_states_;

    //publisher to end-effector state
    ros::Publisher ee_state_publisher_;
    gazebo_msgs::LinkState ee_state_;
    physics::LinkPtr ee_link_;

};

} //namespace gazebo

#endif //PYRAMID_GAZEBO_PLUGINS_GAZEBO_TENSIONS_PLUGIN_H
