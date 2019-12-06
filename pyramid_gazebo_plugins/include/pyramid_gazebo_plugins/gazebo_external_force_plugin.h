#ifndef PYRAMID_GAZEBO_PLUGINS_GAZEBO_EXTERNAL_FORCE_PLUGIN_H
#define PYRAMID_GAZEBO_PLUGINS_GAZEBO_EXTERNAL_FORCE_PLUGIN_H

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

class ExternalForcePlugin : public ModelPlugin
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
    ExternalForcePlugin();
    ~ExternalForcePlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Update();

private:

    void ForceCommandCB(const geometry_msgs::WrenchStampedConstPtr &msg)
    {
        external_force_.force.X() = msg->wrench.force.x;
        external_force_.force.Y() = msg->wrench.force.y;
        external_force_.force.Z() = msg->wrench.force.z;
        external_force_.torque.X() = msg->wrench.torque.x;
        external_force_.torque.Y() = msg->wrench.torque.y;
        external_force_.torque.Z() = msg->wrench.torque.z;
        command_received_ = true;
    }

private:
    ros::NodeHandle rosnode_;
    ros::CallbackQueue callback_queue_;
    event::ConnectionPtr update_event_;
    double update_T_;
    double t_prev_;
    Force external_force_;

    ignition::math::Pose3d pose_;

    physics::ModelPtr model_;

    ros::Subscriber thrust_sub_;
    bool command_received_;

    ros::Publisher ee_state_publisher_;
    gazebo_msgs::LinkState ee_state_;
    physics::LinkPtr ee_link_;

};

} //namespace gazebo

#endif //PYRAMID_GAZEBO_PLUGINS_GAZEBO_EXTERNAL_FORCE_PLUGIN_H
