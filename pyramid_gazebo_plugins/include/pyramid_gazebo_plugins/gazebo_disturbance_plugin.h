#ifndef PYRAMID_GAZEBO_PLUGINS_GAZEBO_DISTURBANCE_PLUGIN_H
#define PYRAMID_GAZEBO_PLUGINS_GAZEBO_DISTURBANCE_PLUGIN_H

#include <random>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ros/callback_queue.h>
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

namespace gazebo
{

class DisturbancePlugin : public ModelPlugin
{
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

public:
    DisturbancePlugin();
    ~DisturbancePlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Update();

private:
    ros::NodeHandle nh_;
    ros::CallbackQueue callback_queue_;
    ros::Subscriber disturbance_sub_;

    event::ConnectionPtr update_event_;
    Force disturbance_;
    bool enable_disturbance_;

    physics::ModelPtr model_;
    physics::LinkPtr base_link_;

    void disturbanceCommandCB(const geometry_msgs::WrenchStampedConstPtr &msg);
};

} //namespace gazebo

#endif //PYRAMID_GAZEBO_PLUGINS_GAZEBO_DISTURBANCE_PLUGIN_H
