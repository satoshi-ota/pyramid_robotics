#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <ros/node_handle.h>

#include "pyramid_gazebo_plugins/gazebo_tensions_plugin.h"

namespace gazebo
{

TensionsPlugin::TensionsPlugin(){ }

TensionsPlugin::~TensionsPlugin()
{
    rosnode_.shutdown();
}

void TensionsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if(!ros::isInitialized)
    {
        ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed.");
        return;
    }

    //get model and name
    model_ = _model;

    //register ROS node & time
    rosnode_ = ros::NodeHandle();
    t_prev_ = 0;

    //initialize
    joints_.clear();
    //tensions_command_.clear();
    rosnode_.param("/use_tether", use_tether_, true);

    if(model_->GetJointCount() != 0 && use_tether_)
    {
        ros::SubscribeOptions opt = ros::SubscribeOptions::create<sensor_msgs::JointState>(
                    "tensions_command", 1,
                    boost::bind(&TensionsPlugin::TensionsCommandCB, this, _1),
                    ros::VoidPtr(),
                    &callback_queue_);
        command_subscriber_ = rosnode_.subscribe(opt);
        //first commans DON'T use in update()
        //use for initialize model configurations
        command_received_ = false;

        // setup joint states
        std::vector<std::string> joint_names;
        std::string name;
        physics::JointPtr joint;

        for(unsigned int i=0;i<model_->GetJointCount();++i)
        {
            joint = model_->GetJoints()[i];
            name = joint->GetName();

            //find prismatic joints
            if(name.find("ugv2tether_joint_prismatic") == 0)
            {
                joints_.push_back(joint);
                // save name
                joint_names.push_back(name);
                // get maximum effort
                f_max = joint->GetEffortLimit(0);
            }
        }
        //publish prismatic joints states
        joint_state_publisher_ = rosnode_.advertise<sensor_msgs::JointState>("tether_states", 1);
        joint_states_.name = joint_names;
        joint_states_.position.resize(joints_.size());
        joint_states_.velocity.resize(joints_.size());
        joint_states_.effort.resize(joints_.size());
    }
    else
    {
        ROS_WARN("set param 'use_tether' as FALSE or invalid model");
    }

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

    //store update rate
    if(_sdf->HasElement("updateRate"))
        update_T_ = 1./_sdf->Get<double>("updateRate");
    else
        update_T_ = 0;

    // Register plugin update
    update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TensionsPlugin::Update, this));

    ros::spinOnce();
    ROS_INFO("Started Tensions Plugin for %s.", _model->GetName().c_str());
}

void TensionsPlugin::Update()
{
    //activate callbacks
    callback_queue_.callAvailable();

    // deal with joint control
    if(command_received_)
    {
        if(use_tether_)
        {
            physics::JointPtr joint;
            unsigned int idx;
            for(unsigned int i=0;i<tensions_command_.name.size();++i)
            {
                // find corresponding model joint
                idx = 0;
                while(joint_states_.name[idx] != tensions_command_.name[i])
                    idx++;
                joint = joints_[idx];
                // only apply positive tensions
                if(tensions_command_.effort[i] > 0)
                    joint->SetForce(0,std::min(tensions_command_.effort[i], f_max));
            }
        }
        else
        {
            ROS_WARN("set param 'use_tether' as TRUE");
        }
    }

    // publish joint states
    double t = ros::Time::now().toSec();
    if((t-t_prev_) > update_T_ && joints_.size() != 0)
    {
        t_prev_ = t;
        joint_states_.header.stamp = ros::Time::now();

        for(unsigned int i=0;i<joints_.size();++i)
        {
            joint_states_.position[i] = joints_[i]->Position();
            joint_states_.velocity[i] = joints_[i]->GetVelocity(0);
            joint_states_.effort[i] = joints_[i]->GetForce(0);
        }
        joint_state_publisher_.publish(joint_states_);
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

GZ_REGISTER_MODEL_PLUGIN(TensionsPlugin);
} //namespace gazebo
