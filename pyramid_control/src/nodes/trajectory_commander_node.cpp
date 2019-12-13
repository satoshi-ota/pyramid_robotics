#include "trajectory_commander_node.h"

namespace pyramid_control
{

TrajectoryCommanderNode::TrajectoryCommanderNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh),
     pos_(Eigen::Vector3d::Zero()),
     att_(Eigen::Vector3d::Zero())
{
    trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>
                        (pyramid_msgs::default_topics::COMMAND_TRAJECTORY, 10);

    srv_ = boost::make_shared
            <dynamic_reconfigure::Server<pyramid_control::TrajectoryGeneratorConfig>>(private_nh);
    dynamic_reconfigure::Server<pyramid_control::TrajectoryGeneratorConfig>::CallbackType cb
        = boost::bind(&TrajectoryCommanderNode::trajectoryReconfig, this, _1, _2);
    srv_->setCallback(cb);
}

TrajectoryCommanderNode::~TrajectoryCommanderNode(){ }

void TrajectoryCommanderNode::trajectoryReconfig(pyramid_control::TrajectoryGeneratorConfig &config,
                                              uint32_t level)
{
    pos_.x() = config.x;
    pos_.y() = config.y;
    pos_.z() = config.z;

    att_.x() = config.roll;
    att_.y() = config.pitch;
    att_.z() = config.yaw;

    semiMinorAxis_ = config.semiMinorAxis;
    semiMajorAxis_ = config.semiMajorAxis;
}

void TrajectoryCommanderNode::targetPosAtt()
{
    pos_.x() = config.x;
    pos_.y() = config.y;
    pos_.z() = config.z;

    att_.x() = config.roll;
    att_.y() = config.pitch;
    att_.z() = config.yaw;
}

void TrajectoryCommanderNode::ellipticOrbit()
{
    t = ros::Time::now();

    pos_.x() = semiMinorAxis_ * cos(t.toSec());
    pos_.y() = semiMajorAxis_ * sin(t.toSec());
    pos_.z() = config.z;

    att_.x() = 0;
    att_.y() = 0;
    att_.z() = 0;
}

void TrajectoryCommanderNode::sendOrbit()
{
    if(trajectory_mode_ == "posatt") targetPosAtt();
    if(trajectory_mode_ == "eliptic") ellipticOrbit();

    trajectory_msg.header.stamp = ros::Time::now();
    pyramid_msgs::msgMultiDofJointTrajectoryFromPosAtt(pos_, att_, &trajectory_msg);
    trajectory_pub_.publish(trajectory_msg);
}

} //namespace pyramid_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_commander_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Duration(1.0).sleep();

    pyramid_control::TrajectoryCommanderNode trajectory_commander_node(nh, private_nh);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        trajectory_commander_node.sendOrbit();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
