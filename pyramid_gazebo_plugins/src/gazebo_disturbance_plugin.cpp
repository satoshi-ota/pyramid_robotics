#include "pyramid_gazebo_plugins/gazebo_disturbance_plugin.h"

namespace gazebo
{

DisturbancePlugin::DisturbancePlugin()
    :enable_disturbance_(true){}

DisturbancePlugin::~DisturbancePlugin()
{
    nh_.shutdown();
}

void DisturbancePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    model_ = _model;

    for(auto &link: model_->GetLinks())
    {
        if(link->GetName() == "pelican/base_link")
            base_link_ = link;
    }

    nh_ = ros::NodeHandle();

    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::WrenchStamped>(
                pyramid_msgs::default_topics::COMMAND_DISTURBANCE, 1,
                boost::bind(&DisturbancePlugin::disturbanceCommandCB, this, _1),
                ros::VoidPtr(), &callback_queue_);
    disturbance_sub_ = nh_.subscribe(ops);

    update_event_ = event::Events::ConnectWorldUpdateBegin
                    (boost::bind(&DisturbancePlugin::Update, this));

    ros::spinOnce();
    ROS_INFO("Started disturbance wind Plugin for %s.", _model->GetName().c_str());
}

void DisturbancePlugin::Update()
{
    callback_queue_.callAvailable();

    base_link_->AddForceAtRelativePosition(disturbance_.force, disturbance_.point);
    base_link_->AddTorque(disturbance_.torque);

    ros::spinOnce();
}

void DisturbancePlugin::disturbanceCommandCB(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    disturbance_.force.X() = msg->wrench.force.x;
    disturbance_.force.Y() = msg->wrench.force.y;
    disturbance_.force.Z() = msg->wrench.force.z;

    disturbance_.torque.X() = msg->wrench.torque.x;
    disturbance_.torque.Y() = msg->wrench.torque.y;
    disturbance_.torque.Z() = msg->wrench.torque.z;
}

GZ_REGISTER_MODEL_PLUGIN(DisturbancePlugin);
} //namespace gazebo
