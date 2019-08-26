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
    srv_ = boost::make_shared <dynamic_reconfigure::Server<pyramid_central::SlidingModeControllerConfig>>( private_nh);
    dynamic_reconfigure::Server<pyramid_central::SlidingModeControllerConfig>::CallbackType cb
        = boost::bind(&SystemCommanderNode::ControllerReconfigureCB, this, _1, _2);
    srv_->setCallback(cb);

    trajectory_sub_ = nh_.subscribe(pyramid_msgs::default_topics::COMMAND_TRAJECTORY, 1,
                                    &SystemCommanderNode::DesiredTrajectoryCB, this);

    odometry_sub_ = nh_.subscribe(pyramid_msgs::default_topics::FEEDBACK_ODOMETRY, 1,
                                  &SystemCommanderNode::FeedbackOdometryCB, this);

    tensions_pub_ = nh_.advertise<sensor_msgs::JointState>
                        (pyramid_msgs::default_topics::COMMAND_TENSIONS, 1);

    thrust_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>
                    (pyramid_msgs::default_topics::COMMAND_THRUST, 1);
}

SystemCommanderNode::~SystemCommanderNode(){ }

void SystemCommanderNode::InitializeParams()
{
    GetSystemParameters(private_nh_, &(sliding_mode_controller_.system_parameters_));

    //initialize tensions_msg
    n_tether_ = sliding_mode_controller_.system_parameters_.n_tether_;

    char tether_name[256];
    for(unsigned int i=0;i<n_tether_;++i)
    {
        sprintf(tether_name, "ugv2tether_joint_prismatic_%i", i);
        tensions_msg.name.push_back(std::string(tether_name));
    }
    tensions_msg.effort.resize(n_tether_);
}

void SystemCommanderNode::ControllerReconfigureCB(
                            pyramid_central::SlidingModeControllerConfig &config,
                            uint32_t level)
{
    system_reconfigure_.SlidingModeControllerReconfig(config,
                                                      &sliding_mode_controller_.system_parameters_);
}

void SystemCommanderNode::DesiredTrajectoryCB(
                            const trajectory_msgs::MultiDOFJointTrajectoryPtr& trajectory_msg)
{
    ROS_INFO_ONCE("Recieved first Desired Trajectory. System controller start!");

    pyramid_msgs::EigenMultiDOFJointTrajectory desired_trajectory;
    pyramid_msgs::eigenMultiDOFJointTrajectoryFromMsg(trajectory_msg, &desired_trajectory);

    sliding_mode_controller_.SetDesiredTrajectory(desired_trajectory);
}

void SystemCommanderNode::FeedbackOdometryCB(const nav_msgs::OdometryPtr& odometry_msg)
{
    ROS_INFO_ONCE("SystemCommander got first feedback odometry msg.");

    pyramid_msgs::EigenOdometry feedback_odometry;
    pyramid_msgs::eigenOdometryFromMsg(odometry_msg, &feedback_odometry);

    sliding_mode_controller_.SetFeedbackOdometry(feedback_odometry);

    sliding_mode_controller_.UpdateTetherDirections();

    sliding_mode_controller_.UpdateDynamicParams();

    sliding_mode_controller_.CalculateSlidingSurface();

    sliding_mode_controller_.CalculateThrust();

    Eigen::VectorXd wrench = sliding_mode_controller_.getWrench();
    Eigen::MatrixXd jacobian = sliding_mode_controller_.getJacobian();
    Eigen::Matrix3d rotation_matrix = sliding_mode_controller_.getRotationMatrix();
    Eigen::Matrix3d to_omega_matrix = sliding_mode_controller_.getToOmegaMatrix();

    tension_distributor_.TensionDistribution(wrench, jacobian, rotation_matrix, to_omega_matrix);

    tension_distributor_.OptimizeTension();

    //publish deisred thrust
    sendThrust();

    //publish desired tensions
    sendTension();
}

void SystemCommanderNode::sendTension()
{
    //write tether tensions
    Eigen::Vector4d desired_tensions = tension_distributor_.getTension();

    tensions_msg.header.stamp = ros::Time::now();

    for(unsigned int i=0;i<n_tether_;++i)
        tensions_msg.effort[i] = desired_tensions(i);

    tensions_pub_.publish(tensions_msg);
}

void SystemCommanderNode::sendThrust()
{
    //write thrust
    Eigen::Vector4d desired_thrust = tension_distributor_.getThrust();

    thrust_msg.header.stamp = ros::Time::now();
    thrust_msg.wrench.force.z = desired_thrust(0);
    thrust_msg.wrench.torque.x = desired_thrust(1);
    thrust_msg.wrench.torque.y = desired_thrust(2);
    thrust_msg.wrench.torque.z = desired_thrust(3);

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
