#include "pyramid_gazebo_plugins/gazebo_winch_plugin.h"

namespace gazebo
{

WinchPlugin::WinchPlugin(){ }

WinchPlugin::~WinchPlugin()
{
    nh_.shutdown();
}

void WinchPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    nh_ = ros::NodeHandle();

    model_ = _model;

    if(_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

    if(_sdf->HasElement("linkName"))
        link_name_ = _sdf->GetElement("linkName")->Get<std::string>();

    if(_sdf->HasElement("winchNumber"))
        winch_number_ = _sdf->GetElement("winchNumber")->Get<int>();

    link_ = model_->GetLink(link_name_);

    ros::SubscribeOptions ops = ros::SubscribeOptions::create<pyramid_msgs::Positions>(
                namespace_ + "/" + pyramid_msgs::default_topics::COMMAND_ANCHOR_POS, 1,
                boost::bind(&WinchPlugin::winchCB, this, _1),
                ros::VoidPtr(), &callback_queue_);
    winch_sub_ = nh_.subscribe(ops);

    update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&WinchPlugin::Update, this));

    ROS_INFO("Started Plugin for %s.", link_name_.c_str());

    ros::spinOnce();
}

void WinchPlugin::Update()
{
    callback_queue_.callAvailable();

    link_->SetWorldPose(pose_, true, true);

    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = link_name_;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = pose_.Pos().X();;
    odom_trans.transform.translation.y = pose_.Pos().Y();;
    odom_trans.transform.translation.z = pose_.Pos().Z();;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

    broadcaster.sendTransform(odom_trans);

    ros::spinOnce();
}

void WinchPlugin::winchCB(const pyramid_msgs::PositionsConstPtr &msg)
{
    pose_.Pos().X() = msg->positions[winch_number_].x;
    pose_.Pos().Y() = msg->positions[winch_number_].y;
    pose_.Pos().Z() = msg->positions[winch_number_].z;
}

GZ_REGISTER_MODEL_PLUGIN(WinchPlugin);
} //namespace gazebo
