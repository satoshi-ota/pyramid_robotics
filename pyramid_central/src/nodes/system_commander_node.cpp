#include "system_commander_node.h"

namespace system_commander
{

SystemCommanderNode::SystemCommanderNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    //get params & initialize
    InitializeParams();

    trajectory_sub_ = nh_.subscribe(pyramid_msgs::default_topics::COMMAND_TRAJECTORY, 1,
                                    &SystemCommanderNode::DesiredTrajectoryCB, this);

    odometry_sub_ = nh_.subscribe(pyramid_msgs::default_topics::FEEDBACK_ODOMETRY, 1,
                                  &SystemCommanderNode::FeedbackOdometryCB, this);

    //imu_sub_ = nh_.subscribe(pyramid_msgs::default_topics::FEEDBACK_IMU, 1,
    //                         &SystemCommanderNode::FeedbackImuCB, this);

    tensions_pub_ = nh_.advertise<sensor_msgs::JointState>
                        (pyramid_msgs::default_topics::COMMAND_TENSIONS, 1);

    thrust_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>
                    (pyramid_msgs::default_topics::COMMAND_THRUST, 1);
}

SystemCommanderNode::~SystemCommanderNode(){ }

void SystemCommanderNode::InitializeParams()
{
    GetSystemParameters(private_nh_, &(system_commander_.system_parameters_));
}

void SystemCommanderNode::DesiredTrajectoryCB(
                            const trajectory_msgs::MultiDOFJointTrajectoryPtr& trajectory_msg)
{
    ROS_INFO_ONCE("Recieved first Desired Trajectory. System controller start!");

    EigenMultiDOFJointTrajectory desired_trajectory;
    eigenMultiDOFJointTrajectoryFromMsg(trajectory_msg, &desired_trajectory);
    system_commander_.SetDesiredTrajectory(desired_trajectory);

    system_commander_.UpdateDynamicParams();

    //acc = acc_d + Kd(vel_d - vel) + Kp(pos_d - pos)
    system_commander_.CalculateInputAcc();

    //calculate controlled variable
    system_commander_.CalculateConrolVariable();

    //publish desired tensions
    sendTensions();

    //publish deisred thrust
    sendThrust();
}

void SystemCommanderNode::FeedbackOdometryCB(const nav_msgs::OdometryPtr& odometry_msg)
{
    ROS_INFO_ONCE("SystemCommander got first feedback odometry msg.");

    EigenOdometry feedback_odometry;
    eigenOdometryFromMsg(odometry_msg, &feedback_odometry);
    system_commander_.SetFeedbackOdometry(feedback_odometry);
}

void SystemCommanderNode::sendTensions()
{
    //write tether tensions

    Eigen::Vector4d desired_tensions = system_commander_.getTensions();

    unsigned int n_tether = system_commander_.system_parameters_.n_tether_;
    tensions_msg.header.stamp = ros::Time::now();
    for(unsigned int i=0;i<n_tether;++i)
        tensions_msg.effort[i] = desired_tensions(i, 0);

    tensions_pub_.publish(tensions_msg);
}

void SystemCommanderNode::sendThrust()
{
    //write thrust
    EigenThrust desired_thrust = system_commander_.getThrust();

    thrust_msg.header.stamp = ros::Time::now();
    eigenThrustToMsg(desired_thrust, thrust_msg);

    thrust_pub_.publish(thrust_msg);
}

} // namespace system_commander

int main(int argc, char** argv) {
  ros::init(argc, argv, "system_commander_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  system_commander::SystemCommanderNode system_commander_node(nh, private_nh);

  ros::spin();

  return 0;
}
