#ifndef PYRAMID_GAZEBO_PLUGINS_GAZEBO_WINCH_PLUGIN_H
#define PYRAMID_GAZEBO_PLUGINS_GAZEBO_WINCH_PLUGIN_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <gazebo_msgs/LinkState.h>
#include <pyramid_msgs/Positions.h>
#include <tf/transform_broadcaster.h>
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

class WinchPlugin : public ModelPlugin
{
public:
    WinchPlugin();
    ~WinchPlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Update();

private:
    ros::NodeHandle nh_;
    ros::Subscriber winch_sub_;
    ros::CallbackQueue callback_queue_;
    event::ConnectionPtr update_event_;

    tf::TransformBroadcaster broadcaster;

    physics::ModelPtr model_;
    physics::LinkPtr link_;

    ignition::math::Pose3d pose_;
    geometry_msgs::TransformStamped odom_trans;

    std::string namespace_;
    std::string link_name_;
    int winch_number_;

    void winchCB(const pyramid_msgs::PositionsConstPtr &msg);
};

} //namespace gazebo

#endif //PYRAMID_GAZEBO_PLUGINS_GAZEBO_WINCH_PLUGIN_H
