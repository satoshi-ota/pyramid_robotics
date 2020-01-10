#include "pyramid_gazebo_plugins/gazebo_external_force_plugin.h"

namespace gazebo
{

ExternalForcePlugin::ExternalForcePlugin(){ }

ExternalForcePlugin::~ExternalForcePlugin()
{
    nh_.shutdown();
}

void ExternalForcePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    model_ = _model;

    nh_ = ros::NodeHandle();

    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::WrenchStamped>(
                pyramid_msgs::default_topics::COMMAND_THRUST, 1,
                boost::bind(&ExternalForcePlugin::ForceCommandCB, this, _1),
                ros::VoidPtr(), &callback_queue_);
    thrust_sub_ = nh_.subscribe(ops);

    for(auto &link: model_->GetLinks())
    {
        if(link->GetName() == "pelican/base_link")
            link_ = link;
    }

    update_event_
        = event::Events::ConnectWorldUpdateBegin(boost::bind(&ExternalForcePlugin::Update, this));

    command_received_ = false;

    ROS_INFO("Started external force Plugin for %s.", _model->GetName().c_str());

    ros::spinOnce();
}

void ExternalForcePlugin::Update()
{
    callback_queue_.callAvailable();

    if(command_received_ == true)
    {
        link_->AddForce(external_force_.force);
        link_->AddTorque(external_force_.torque);
    }

    ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(ExternalForcePlugin);
} //namespace gazebo
