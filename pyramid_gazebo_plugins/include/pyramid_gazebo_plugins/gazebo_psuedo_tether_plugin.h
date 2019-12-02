#ifndef PYRAMID_GAZEBO_PLUGINS_GAZEBO_PSUEDO_TETHER_PLUGIN_H
#define PYRAMID_GAZEBO_PLUGINS_GAZEBO_PSUEDO_TETHER_PLUGIN_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pyramid_msgs/default_topics.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>

namespace gazebo{

class PsuedoTetherPlugin : public ModelPlugin
{
public:
    PsuedoTetherPlugin();
    ~PsuedoTetherPlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Update();

private: //data member
    //general
    ros::NodeHandle rosnode_;
    event::ConnectionPtr update_event_;
    double update_T_;
    double t_prev_;
    Force external_force_;

    //model
    physics::ModelPtr model_;

    //publisher to end-effector state
    ros::Publisher direc_pub_;
    std::vector<geometry_msgs::Point> winch_, tether_end_;

};

} //namespace gazebo

#endif //PYRAMID_GAZEBO_PLUGINS_GAZEBO_PSUEDO_TETHER_PLUGIN_H
