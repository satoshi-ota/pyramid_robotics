#include "sliding_mode_control_node.h"

namespace system_commander
{

SlidingModeControlNode::SlidingModeControlNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh),
     begin_(ros::Time::now())
{
    InitializeParams();

    srv_ = boost::make_shared
            <dynamic_reconfigure::Server<pyramid_central::SlidingModeControllerConfig>>( private_nh);
    dynamic_reconfigure::Server<pyramid_central::SlidingModeControllerConfig>::CallbackType cb
        = boost::bind(&SlidingModeControlNode::ControllerReconfigureCB, this, _1, _2);
    srv_->setCallback(cb);


    trajectory_sub_ = nh_.subscribe(pyramid_msgs::default_topics::COMMAND_TRAJECTORY, 1,
                                    &SlidingModeControlNode::DesiredTrajectoryCB, this);

    odometry_sub_ = nh_.subscribe(pyramid_msgs::default_topics::FEEDBACK_ODOMETRY, 1,
                                  &SlidingModeControlNode::FeedbackOdometryCB, this);

    thrust_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>
                    (pyramid_msgs::default_topics::COMMAND_THRUST, 1);
}

SlidingModeControlNode::~SlidingModeControlNode(){ }

void SlidingModeControlNode::InitializeParams()
{
    GetSystemParameters(private_nh_, &(sliding_mode_controller_.system_parameters_));
}

void SlidingModeControlNode::ControllerReconfigureCB(
                                pyramid_central::SlidingModeControllerConfig &config,           uint32_t level)
{
    system_reconfigure_.SlidingModeControllerReconfig(config,
                                                      &sliding_mode_controller_.system_parameters_);
}

void SlidingModeControlNode::DesiredTrajectoryCB(
                            const trajectory_msgs::MultiDOFJointTrajectoryPtr& trajectory_msg)
{
    ROS_INFO_ONCE("Recieved first Desired Trajectory. SMC START!");

    pyramid_msgs::EigenMultiDOFJointTrajectory desired_trajectory;
    pyramid_msgs::eigenMultiDOFJointTrajectoryFromMsg(trajectory_msg, &desired_trajectory);

    sliding_mode_controller_.SetDesiredTrajectory(desired_trajectory);
}

void SlidingModeControlNode::FeedbackOdometryCB(const nav_msgs::OdometryPtr& odometry_msg)
{
    ROS_INFO_ONCE("Recieved first odometry msg.");

    pyramid_msgs::EigenOdometry feedback_odometry;
    pyramid_msgs::eigenOdometryFromMsg(odometry_msg, &feedback_odometry);

    sliding_mode_controller_.SetFeedbackOdometry(feedback_odometry);

    sliding_mode_controller_.UpdateDynamicParams();

    sliding_mode_controller_.CalculateSlidingSurface();

    sliding_mode_controller_.CalculateThrust();

    sendThrust();
}

void SlidingModeControlNode::sendThrust()
{
    //write thrust
    pyramid_msgs::EigenThrust desired_thrust = sliding_mode_controller_.getThrust();

    thrust_msg.header.stamp = ros::Time::now();
    pyramid_msgs::eigenThrustToMsg(desired_thrust, thrust_msg);

    thrust_pub_.publish(thrust_msg);
}

} // namespace system_commander

int main(int argc, char** argv) {
  ros::init(argc, argv, "liding_mode_control_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  system_commander::SlidingModeControlNode sliding_mode_control_node(nh, private_nh);

  ros::spin();

  return 0;
}
