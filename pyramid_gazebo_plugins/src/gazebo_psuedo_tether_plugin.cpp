#include "pyramid_gazebo_plugins/gazebo_psuedo_tether_plugin.h"

namespace gazebo
{

PsuedoTetherPlugin::PsuedoTetherPlugin(){ }

PsuedoTetherPlugin::~PsuedoTetherPlugin()
{
    rosnode_.shutdown();
}

void PsuedoTetherPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if(!ros::isInitialized)
    {
        ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed.");
        return;
    }

    model_ = _model;
    world_ = model_->GetWorld();

    rosnode_ = ros::NodeHandle();

    getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_, true);

    link_ = model_->GetLink(link_name_);

    child_links_ = link_->GetChildJointsLinks();
    for(int i = 0; i < child_links_.size(); i++)
    {

    }




    direc_pub_ = rosnode_.advertise<geometry_msgs::Vector3>("tether_direction",1);

    // Register plugin update
    update_event_
        = event::Events::ConnectWorldUpdateBegin(boost::bind(&PsuedoTetherPlugin::Update, this));

    ros::spinOnce();
    ROS_INFO("Started PSUEDO TETHER Plugin for %s.", _model->GetName().c_str());
}

void PsuedoTetherPlugin::Update()
{
    // tension_.force.X() = 0;
    // tension_.force.Y() = 5;
    // tension_.force.Z() = 15;
    // //tension_.torque.Z() = 10;
    // link_->AddForce(tension_.force);
    //link_->AddTorque(tension_.torque);
    // if(command_received_)
    // {
    //     ee_link_->AddForceAtRelativePosition(external_force_.force, external_force_.point);
    //     ee_link_->AddTorque(external_force_.torque);
    // }

    // auto ee_pose = ee_link_->WorldPose() - ee_link_->WorldPose();
    // ee_state_.pose.position.x = ee_pose.Pos().X();
    // ee_state_.pose.position.y = ee_pose.Pos().Y();
    // ee_state_.pose.position.z = ee_pose.Pos().Z();
    // ee_state_.pose.orientation.x = ee_pose.Rot().X();
    // ee_state_.pose.orientation.y = ee_pose.Rot().Y();
    // ee_state_.pose.orientation.z = ee_pose.Rot().Z();
    // ee_state_.pose.orientation.w = ee_pose.Rot().W();
    // auto vel = ee_pose.Rot().RotateVector(ee_link_->RelativeLinearVel());
    // ee_state_.twist.linear.x = vel.X();
    // ee_state_.twist.linear.y = vel.Y();
    // ee_state_.twist.linear.z = vel.Z();
    // vel = ee_pose.Rot().RotateVector(ee_link_->RelativeAngularVel());
    // ee_state_.twist.angular.x = vel.X();
    // ee_state_.twist.angular.y = vel.Y();
    // ee_state_.twist.angular.z = vel.Z();
    //
    // ee_state_publisher_.publish(ee_state_);
    ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(PsuedoTetherPlugin);
} //namespace gazebo
