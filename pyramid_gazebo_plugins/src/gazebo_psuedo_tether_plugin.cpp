#include "pyramid_gazebo_plugins/gazebo_psuedo_tether_plugin.h"

namespace gazebo
{

PsuedoTetherPlugin::PsuedoTetherPlugin(){ }

PsuedoTetherPlugin::~PsuedoTetherPlugin()
{
    nh_.shutdown();
}

void PsuedoTetherPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    nh_ = ros::NodeHandle();

    ros::SubscribeOptions ops = ros::SubscribeOptions::create<pyramid_msgs::Tensions>(
                pyramid_msgs::default_topics::COMMAND_TENSIONS, 1,
                boost::bind(&PsuedoTetherPlugin::TensionCommandCB, this, _1),
                ros::VoidPtr(), &callback_queue_);
    tension_sub_ = nh_.subscribe(ops);

    model_ = _model;

    if(_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

    if(_sdf->HasElement("linkName"))
        link_name_ = _sdf->GetElement("linkName")->Get<std::string>();

    if(_sdf->HasElement("tetherNumber"))
        tether_number_ = _sdf->GetElement("tetherNumber")->Get<int>();

    frame_id_ = link_name_;

    link_ = model_->GetLink(link_name_);

    update_event_
        = event::Events::ConnectWorldUpdateBegin(boost::bind(&PsuedoTetherPlugin::Update, this));

    command_received_ = false;

    ROS_INFO("Started PSUEDO TETHER Plugin for %s.", link_->GetName().c_str());

    ros::spinOnce();
}

void PsuedoTetherPlugin::Update()
{
    callback_queue_.callAvailable();

    if(command_received_ == true) link_->AddForce(tension_.force);

    ros::spinOnce();
}

void PsuedoTetherPlugin::TensionCommandCB(const pyramid_msgs::TensionsConstPtr &msg)
{
    command_received_ = true;

    tension_.force.X() = msg->tensions[tether_number_].x;
    tension_.force.Y() = msg->tensions[tether_number_].y;
    tension_.force.Z() = msg->tensions[tether_number_].z;
}

GZ_REGISTER_MODEL_PLUGIN(PsuedoTetherPlugin);
} //namespace gazebo
