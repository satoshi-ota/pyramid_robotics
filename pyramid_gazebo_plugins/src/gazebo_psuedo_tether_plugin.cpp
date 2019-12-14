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

    marker_pub_  = nh_.advertise<visualization_msgs::MarkerArray>
                   (pyramid_msgs::default_topics::MARKER_TENSION + link_name_, 1);

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

    ignition::math::Vector3d start(link_->WorldPose().Pos());

    geometry_msgs::Point linear_start;
    linear_start.x = start.X();
    linear_start.y = start.Y();
    linear_start.z = start.Z();

    geometry_msgs::Point linear_end;
    linear_end.x = start.X() + tension_.force.X();
    linear_end.y = start.Y() + tension_.force.Y();
    linear_end.z = start.Z() + tension_.force.Z();

    geometry_msgs::Vector3 arrow;
    arrow.x = 0.02;
    arrow.y = 0.04;
    arrow.z = 0.1;

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(1);

    marker_array.markers[0].header.frame_id = "world";
    marker_array.markers[0].header.stamp = ros::Time::now();
    marker_array.markers[0].ns = namespace_;
    marker_array.markers[0].id = tether_number_;
    marker_array.markers[0].lifetime = ros::Duration();

    marker_array.markers[0].type = visualization_msgs::Marker::ARROW;
    marker_array.markers[0].action = visualization_msgs::Marker::ADD;
    marker_array.markers[0].scale = arrow;

    marker_array.markers[0].points.resize(2);
    marker_array.markers[0].points[0] = linear_start;
    marker_array.markers[0].points[1] = linear_end;

    marker_array.markers[0].pose.orientation.x = 0.0;
    marker_array.markers[0].pose.orientation.y = 0.0;
    marker_array.markers[0].pose.orientation.z = 0.0;
    marker_array.markers[0].pose.orientation.w = 1.0;

    marker_array.markers[0].color.r = 0.0f;
    marker_array.markers[0].color.g = 1.0f;
    marker_array.markers[0].color.b = 0.0f;
    marker_array.markers[0].color.a = 0.5f;

    marker_pub_.publish(marker_array);
}

GZ_REGISTER_MODEL_PLUGIN(PsuedoTetherPlugin);
} //namespace gazebo
