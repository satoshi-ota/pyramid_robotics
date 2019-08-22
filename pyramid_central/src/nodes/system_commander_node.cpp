#include "system_commander_node.h"

namespace system_commander
{

SystemCommanderNode::SystemCommanderNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh),
     begin_(ros::Time::now())
{
    //get params & initialize
    InitializeParams();

    //set up dynamic reconfigure
    srv_ = boost::make_shared <dynamic_reconfigure::Server<pyramid_central::SystemCommanderConfig>>( private_nh);
    dynamic_reconfigure::Server<pyramid_central::SystemCommanderConfig>::CallbackType cb
        = boost::bind(&SystemCommanderNode::ControllerReconfigureCB, this, _1, _2);
    srv_->setCallback(cb);

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

    //initialize tensions_msg
    n_tether_ = system_commander_.system_parameters_.n_tether_;

    char tether_name[256];
    for(unsigned int i=0;i<n_tether_;++i)
    {
        sprintf(tether_name, "ugv2tether_joint_prismatic_%i", i);
        tensions_msg.name.push_back(std::string(tether_name));
    }
    tensions_msg.effort.resize(n_tether_);
}

void SystemCommanderNode::ControllerReconfigureCB(pyramid_central::SystemCommanderConfig &config,
                                        uint32_t level)
{
    system_reconfigure_.ControllerReconfig(config, &system_commander_.system_parameters_);
}

void SystemCommanderNode::DesiredTrajectoryCB(
                            const trajectory_msgs::MultiDOFJointTrajectoryPtr& trajectory_msg)
{
    ROS_INFO_ONCE("Recieved first Desired Trajectory. System controller start!");

    EigenMultiDOFJointTrajectory desired_trajectory;
    eigenMultiDOFJointTrajectoryFromMsg(trajectory_msg, &desired_trajectory);

    system_commander_.SetDesiredTrajectory(desired_trajectory);
}

void SystemCommanderNode::FeedbackOdometryCB(const nav_msgs::OdometryPtr& odometry_msg)
{
    ROS_INFO_ONCE("SystemCommander got first feedback odometry msg.");

    EigenOdometry feedback_odometry;
    eigenOdometryFromMsg(odometry_msg, &feedback_odometry);

    system_commander_.SetFeedbackOdometry(feedback_odometry);

    system_commander_.UpdateTetherDirections();

    system_commander_.UpdateDynamicParams();

    //acc = acc_d + Kd(vel_d - vel) + Kp(pos_d - pos)
    system_commander_.CalculateInputAcc();

    //calculate controlled variable
    system_commander_.CalculateConrolVariable();

    //publish deisred thrust
    sendThrust();

    //publish desired tensions
    sendTensions();
}

void SystemCommanderNode::sendTensions()
{
    //write tether tensions
    Eigen::Vector4d desired_tensions = system_commander_.getTensions();

    tensions_msg.header.stamp = ros::Time::now();

    for(unsigned int i=0;i<n_tether_;++i)
        tensions_msg.effort[i] = desired_tensions(i);

    double duration = (ros::Time::now() - begin_).toSec();

    //wait for uav take off
    //if(duration > 10.0)
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
