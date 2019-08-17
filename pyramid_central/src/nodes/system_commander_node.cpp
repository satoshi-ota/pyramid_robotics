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
                                    &SystemCommanderNode::DesiredTrajectryCB, this);

    odometry_sub_ = nh_.subscribe(pyramid_msgs::default_topics::FEEDBACK_ODOMETRY, 1,
                                  &SystemCommanderNode::FeedbackOdometryCB, this);

    imu_sub_ = nh_.subscribe(pyramid_msgs::default_topics::FEEDBACK_IMU, 1,
                             &SystemCommanderNode::FeedbackImuCB, this);

    tensions_pub_ = nh_.advertise<sensor_msgs::JointState>
                        (pyramid_msgs::default_topics::COMMAND_TENSIONS, 1);

    thrust_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>
                    (pyramid_msgs::default_topics::COMMAND_THRUST, 1);
}

SystemCommanderNode::~SystemCommanderNode(){ }

void SystemCommanderNode::InitializeParams()
{
    GetVehicleParameters(private_nh_, &system_commander_.vehicle_parameters_);
}

void DesiredTrajectryCB(const trajectry_msgs::MultiDOFJointTrajectryPtr& trajectry_msg)
{
    ROS_INFO_ONCE("Recieved first Desired Trajectry. System controller start!")

    EigenMultiDOFJointTrajectry desired_trajectry;
    eigenMultiDOFJointTrajectryFromMsg(trajectry_msg, &desired_trajectry);
    system_commander_.SetDesiredTrajectry(desired_trajectry);

    system_commander_.CalculateSystemDynamicsParameters();

    //acc = acc_d + Kd(vel_d - vel) + Kp(pos_d - pos)
    system_commander_.CalculateInputAcc();

    //calculate controlled variable
    system_commander_.CalculateConrolVariable();

    //publish desired tensions
    Eigen::VectorXd desired_tensions;
    desired_tensions = system_commander_.getTensions();
    sendTensions(desired_tensions);

    //publish deisred thrust
    Eigen::VectorXd desired_thrust;
    desired_thrust = system_parameters_.getThrust();
    sendThrust(desired_thrust);
}

void SystemCommanderNode::FeedbackOdometryCB(const geometry_msgs::OdometryPtr& odometry_msg)
{
    ROS_INFO_ONCE("SystemCommander got first feedback odometry msg.")

    EigenOdometry feedback_odometry;
    eigenOdometryFromMsg(odometry_msg, &feedback_odometry);
    system_commander_.SetFeedbackOdometry(feedback_odometry);
}

void SystemCommanderNode::sendTensions(Eigen::VectorXd& tensions)
{
    //write tether tensions
    tensions_msg.header.stamp = ros::Time::now();
    for(unsigned int i=0;i<n_tether_;++i)
        tensions_msg.effort[i] = tensions(i, 0);

    tensions_pub_.publish(tensions_msg);
}

void SystemCommanderNode::sendThrust(Eigen::VectorXd& thrust)
{
    //write thrust


    thrust_pub_.publish(thrust_msg);
}

} // namespace system_commander
