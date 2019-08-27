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

    void DisturbanceGen()
    {
        t_ = ros::Time::now();
        std::normal_distribution<> x_dist_(x_mu_, x_sig_);
        std::normal_distribution<> y_dist_(y_mu_, y_sig_);
        std::mt19937 x_engine_(x_seed_());
        std::mt19937 y_engine_(y_seed_());
        force_.force.X() = 3*sin(2*M_PI/10*t_.toSec()) + x_dist_(x_engine_);
        force_.force.Y() = 3*sin(2*M_PI/10*t_.toSec()) + y_dist_(y_engine_);
        force_.force.Z() = 0.0;
    }

private: //data member
    //general
    ros::NodeHandle nh_;
    ros::Time t_;
    event::ConnectionPtr update_event_;
    Force force_;
    bool enable_disturbance_;

    //normal_distribution
    //std::mt19937 x_engine_, y_engine_;
    std::random_device x_seed_, y_seed_;
    double x_mu_, x_sig_, y_mu_, y_sig_;

    physics::ModelPtr model_;

    ros::Publisher disturbance_pub_;
    geometry_msgs::WrenchStamped disturbance_;
    physics::LinkPtr base_link_;
};

} //namespace gazebo

#endif //PYRAMID_GAZEBO_PLUGINS_GAZEBO_DISTURBANCE_PLUGIN_H
