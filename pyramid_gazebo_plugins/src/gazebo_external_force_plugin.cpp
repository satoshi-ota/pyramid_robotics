#include "pyramid_gazebo_plugins/gazebo_external_force_plugin.h"
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

namespace gazebo
{

ExternalForcePlugin::ExternalForcePlugin(){ }

ExternalForcePlugin::~ExternalForcePlugin()
{
    rosnode_.shutdown();
}

void ExternalForcePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if(!ros::isInitialized)
    {
        ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed.");
        return;
    }

    model_ = _model;

    rosnode_ = ros::NodeHandle();
    t_prev_ = 0;

    // init subscriber
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::WrenchStamped>(
                pyramid_msgs::default_topics::COMMAND_THRUST, 1,
                boost::bind(&ExternalForcePlugin::ForceCommandCB, this, _1),
                ros::VoidPtr(), &callback_queue_);
    thrust_sub_ = rosnode_.subscribe(ops);
    command_received_ = false;

    //find end-effector links
    for(auto &link: model_->GetLinks())
    {
        //"pelican" is uav
        if(link->GetName() == "pelican/base_link")
            ee_link_ = link;
    }

    //publish end-effector state
    ee_state_publisher_ = rosnode_.advertise<gazebo_msgs::LinkState>("end_effector_state",1);
    ee_state_.link_name = "end_effector";
    ee_state_.reference_frame = "world";

    // Register plugin update
    update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ExternalForcePlugin::Update, this));

    ros::spinOnce();
    ROS_INFO("Started external force Plugin for %s.", _model->GetName().c_str());
}

void ExternalForcePlugin::Update()
{
    callback_queue_.callAvailable();

    if(command_received_)
    {
        ee_link_->AddForce(external_force_.force);
        ee_link_->AddTorque(external_force_.torque);
    }

    auto ee_pose = ee_link_->WorldPose() - ee_link_->WorldPose();
    ee_state_.pose.position.x = ee_pose.Pos().X();
    ee_state_.pose.position.y = ee_pose.Pos().Y();
    ee_state_.pose.position.z = ee_pose.Pos().Z();
    ee_state_.pose.orientation.x = ee_pose.Rot().X();
    ee_state_.pose.orientation.y = ee_pose.Rot().Y();
    ee_state_.pose.orientation.z = ee_pose.Rot().Z();
    ee_state_.pose.orientation.w = ee_pose.Rot().W();
    auto vel = ee_pose.Rot().RotateVector(ee_link_->RelativeLinearVel());
    ee_state_.twist.linear.x = vel.X();
    ee_state_.twist.linear.y = vel.Y();
    ee_state_.twist.linear.z = vel.Z();
    vel = ee_pose.Rot().RotateVector(ee_link_->RelativeAngularVel());
    ee_state_.twist.angular.x = vel.X();
    ee_state_.twist.angular.y = vel.Y();
    ee_state_.twist.angular.z = vel.Z();

    ee_state_publisher_.publish(ee_state_);
    ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(ExternalForcePlugin);
} //namespace gazebo
