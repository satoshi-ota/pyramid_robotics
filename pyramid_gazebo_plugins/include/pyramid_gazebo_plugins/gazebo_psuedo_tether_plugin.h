#ifndef PYRAMID_GAZEBO_PLUGINS_GAZEBO_PSUEDO_TETHER_PLUGIN_H
#define PYRAMID_GAZEBO_PLUGINS_GAZEBO_PSUEDO_TETHER_PLUGIN_H

#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <gazebo_msgs/LinkState.h>
#include <pyramid_msgs/default_topics.h>
#include <pyramid_msgs/Tensions.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>

namespace gazebo{

struct Force
{
    Force()
    :point(ignition::math::Vector3d(0, 0, 0)),
     force(ignition::math::Vector3d(0, 0, 0)),
     torque(ignition::math::Vector3d(0, 0, 0)){ }

    ignition::math::Vector3d point;
    ignition::math::Vector3d force;
    ignition::math::Vector3d torque;
    std::string name;
};

class PsuedoTetherPlugin : public ModelPlugin
{
public:
    PsuedoTetherPlugin();
    ~PsuedoTetherPlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Update();

private:
    ros::NodeHandle nh_;
    ros::CallbackQueue callback_queue_;
    event::ConnectionPtr update_event_;

    ros::Subscriber tension_sub_;

    physics::ModelPtr model_;
    physics::LinkPtr link_;

    std::string namespace_;
    std::string link_name_;
    std::string frame_id_;
    int tether_number_;
    bool command_received_;

    Force tension_;

    void TensionCommandCB(const pyramid_msgs::TensionsConstPtr &msg);
};

} //namespace gazebo

#endif //PYRAMID_GAZEBO_PLUGINS_GAZEBO_PSUEDO_TETHER_PLUGIN_H
