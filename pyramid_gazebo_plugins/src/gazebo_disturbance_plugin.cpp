#include "pyramid_gazebo_plugins/gazebo_disturbance_plugin.h"

namespace gazebo
{

DisturbancePlugin::DisturbancePlugin()
    :enable_disturbance_(true),
     t_(ros::Time::now()),
     x_mu_(0.0),
     x_sig_(0.5),
     y_mu_(0.0),
     y_sig_(0.5){ }

DisturbancePlugin::~DisturbancePlugin()
{
    nh_.shutdown();
}

void DisturbancePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if(!ros::isInitialized)
    {
        ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed.");
        return;
    }

    //get model and name
    model_ = _model;

    for(auto &link: model_->GetLinks())
    {
        if(link->GetName() == "pelican/base_link")
            base_link_ = link;
    }

    //register ROS node & time
    nh_ = ros::NodeHandle();

    disturbance_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>
                                    (pyramid_msgs::default_topics::DISTURBANCE,1);

    // Register plugin update
    update_event_ = event::Events::ConnectWorldUpdateBegin
                                        (boost::bind(&DisturbancePlugin::Update, this));

    ros::spinOnce();
    ROS_INFO("Started disturbance wind Plugin for %s.", _model->GetName().c_str());
}

void DisturbancePlugin::Update()
{
    DisturbanceGen();

    if(enable_disturbance_)
    {
        base_link_->AddForceAtRelativePosition(force_.force, force_.point);
        base_link_->AddTorque(force_.torque);
    }

    disturbance_.header.stamp = ros::Time::now();
    disturbance_.header.frame_id = "pelican/base_link";
    disturbance_.wrench.force.x = force_.force.X();
    disturbance_.wrench.force.y = force_.force.Y();
    disturbance_.wrench.force.z = force_.force.Z();
    disturbance_.wrench.torque.x = force_.torque.X();
    disturbance_.wrench.torque.y = force_.torque.Y();
    disturbance_.wrench.torque.z = force_.torque.Z();
    disturbance_pub_.publish(disturbance_);

    ros::spinOnce();
}
GZ_REGISTER_MODEL_PLUGIN(DisturbancePlugin);
} //namespace gazebo
