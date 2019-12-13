#include "pyramid_gazebo_plugins/gazebo_winch_plugin.h"
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

namespace gazebo
{

WinchPlugin::WinchPlugin(){ }

WinchPlugin::~WinchPlugin()
{
    nh_.shutdown();
}

void WinchPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if(!ros::isInitialized)
    {
        ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed.");
        return;
    }

    model_ = _model;

    nh_ = ros::NodeHandle();

    if(_sdf->HasElement("linkName"))
    {
        link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    }
    else
    {
        // robot_description_ = "robot_description"; // default
    }

    link_ = model_->GetLink(link_name_);


    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = link_name_;

    update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&WinchPlugin::Update, this));

    ros::spinOnce();
    ROS_INFO("Started Plugin for %s.", link_name_.c_str());
}

void WinchPlugin::Update()
{
    auto pose = link_->WorldPose();

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = pose.Pos().X();;
    odom_trans.transform.translation.y = pose.Pos().Y();;
    odom_trans.transform.translation.z = pose.Pos().Z();;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

    broadcaster.sendTransform(odom_trans);
    ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(WinchPlugin);
} //namespace gazebo
