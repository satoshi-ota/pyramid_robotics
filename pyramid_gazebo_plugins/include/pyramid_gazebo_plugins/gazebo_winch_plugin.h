#ifndef PYRAMID_GAZEBO_PLUGINS_GAZEBO_WINCH_PLUGIN_H
#define PYRAMID_GAZEBO_PLUGINS_GAZEBO_WINCH_PLUGIN_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pyramid_msgs/default_topics.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>

namespace gazebo{

class WinchPlugin : public ModelPlugin
{
public:
    WinchPlugin();
    ~WinchPlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Update();

private:
    ros::NodeHandle nh_;
    event::ConnectionPtr update_event_;

    ignition::math::Pose3d pose_;

    physics::ModelPtr model_;

    std::string link_name_;

    ros::Publisher state_publisher_;
    sensor_msgs::JointState state_;
    physics::LinkPtr link_;

    tf::TransformBroadcaster broadcaster;

    geometry_msgs::TransformStamped odom_trans;

};

} //namespace gazebo

#endif //PYRAMID_GAZEBO_PLUGINS_GAZEBO_WINCH_PLUGIN_H
