#include "trajectory_commander_node.h"

namespace system_commander
{

TrajectoryCommanderNode::TrajectoryCommanderNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh),
     desired_position_(Eigen::Vector3d::Zero()),
     desired_yaw_(0.0)
{
    trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>
                        (pyramid_msgs::default_topics::COMMAND_TRAJECTORY, 10);

    //set up dynamic reconfigure
    srv_ = boost::make_shared
            <dynamic_reconfigure::Server<pyramid_central::TrajectoryGeneratorConfig>>( private_nh);
    dynamic_reconfigure::Server<pyramid_central::TrajectoryGeneratorConfig>::CallbackType cb
        = boost::bind(&TrajectoryCommanderNode::SetTrajectoryCB, this, _1, _2);
    srv_->setCallback(cb);
}

TrajectoryCommanderNode::~TrajectoryCommanderNode(){ }

void TrajectoryCommanderNode::SetTrajectoryCB(pyramid_central::TrajectoryGeneratorConfig &config,
                                              uint32_t level)
{
    system_reconfigure_.TrajectoryReconfig(config);

    trajectory_msg.header.stamp = ros::Time::now();

    desired_position_ = system_reconfigure_.getDesiredPosition();
    desired_yaw_ = system_reconfigure_.getDesiredYaw();

    pyramid_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position_,
                                                            desired_yaw_, &trajectory_msg);

    ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
             nh_.getNamespace().c_str(), desired_position_.x(),
             desired_position_.y(), desired_position_.z());

    trajectory_pub_.publish(trajectory_msg);
}

} //namespace system_commander

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_commander_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ROS_INFO("Started trajectory_commander_node.");

    ros::Duration(5.0).sleep();

    system_commander::TrajectoryCommanderNode trajectory_commander_node(nh, private_nh);

    ros::spin();

    return 0;
}
