#include "sliding_mode_control_node.h"

namespace pyramid_control
{

SlidingModeControlNode::SlidingModeControlNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    ROS_INFO_ONCE("GOOD41");
    GetSystemParameters(private_nh_, &(sliding_mode_controller_.system_parameters_));

    srv_ = boost::make_shared
            <dynamic_reconfigure::Server<pyramid_control::SlidingModeControllerConfig>>( private_nh);
    dynamic_reconfigure::Server<pyramid_control::SlidingModeControllerConfig>::CallbackType cb
        = boost::bind(&SlidingModeControlNode::reconfigureCB, this, _1, _2);
    srv_->setCallback(cb);


    trajectory_sub_ = nh_.subscribe(pyramid_msgs::default_topics::COMMAND_TRAJECTORY, 1,
                                    &SlidingModeControlNode::trajectoryCB, this);

    odometry_sub_ = nh_.subscribe(pyramid_msgs::default_topics::FEEDBACK_ODOMETRY, 1,
                                  &SlidingModeControlNode::odometryCB, this);

    thrust_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>
                    (pyramid_msgs::default_topics::COMMAND_THRUST, 1);
    ROS_INFO_ONCE("GOOD42");
}

SlidingModeControlNode::~SlidingModeControlNode(){ }

void SlidingModeControlNode::reconfigureCB(pyramid_control::SlidingModeControllerConfig &config,
                                           uint32_t level)
{
    system_reconfigure_.smcReconfig(config, &sliding_mode_controller_.system_parameters_);
}

void SlidingModeControlNode::trajectoryCB(
                            const trajectory_msgs::MultiDOFJointTrajectoryPtr& trajectory_msg)
{
    ROS_INFO_ONCE("Recieved first Desired Trajectory. SMC START!");

    pyramid_msgs::EigenMultiDOFJointTrajectory trajectory;
    pyramid_msgs::eigenMultiDOFJointTrajectoryFromMsg(trajectory_msg, &trajectory);

    sliding_mode_controller_.setTrajectory(trajectory);
}

void SlidingModeControlNode::odometryCB(const nav_msgs::OdometryPtr& odometry_msg)
{
    ROS_INFO_ONCE("Recieved first odometry msg.");

    pyramid_msgs::EigenOdometry odometry;
    pyramid_msgs::eigenOdometryFromMsg(odometry_msg, &odometry);

    sliding_mode_controller_.setOdometry(odometry);
    sliding_mode_controller_.updateModelConfig();
    sliding_mode_controller_.calcThrust();

    sendThrust();
}

void SlidingModeControlNode::sendThrust()
{
    geometry_msgs::WrenchStamped thrust_msg;

    pyramid_msgs::EigenThrust thrust = sliding_mode_controller_.getThrust();
    thrust_msg.header.stamp = ros::Time::now();
    pyramid_msgs::eigenThrustToMsg(thrust, thrust_msg);

    thrust_pub_.publish(thrust_msg);
}

} // namespace pyramid_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "sliding_mode_control_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  pyramid_control::SlidingModeControlNode sliding_mode_control_node(nh, private_nh);

  ros::spin();

  return 0;
}
