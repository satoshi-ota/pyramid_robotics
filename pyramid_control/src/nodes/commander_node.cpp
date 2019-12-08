#include "commander_node.h"

namespace pyramid_control
{

CommanderNode::CommanderNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    GetSystemParameters(private_nh_, &(sliding_mode_controller_.system_parameters_));
    GetSystemParameters(private_nh_, &(actuator_controller_.system_parameters_));

    actuator_controller_.InitializeParameters();

    srv_ = boost::make_shared
            <dynamic_reconfigure::Server<pyramid_control::SlidingModeControllerConfig>>(private_nh);
    dynamic_reconfigure::Server<pyramid_control::SlidingModeControllerConfig>::CallbackType cb
        = boost::bind(&CommanderNode::paramsReconfig, this, _1, _2);
    srv_->setCallback(cb);


    trajectory_sub_ = nh_.subscribe(pyramid_msgs::default_topics::COMMAND_TRAJECTORY, 1,
                                    &CommanderNode::trajectoryCB, this);

    odometry_sub_ = nh_.subscribe(pyramid_msgs::default_topics::FEEDBACK_ODOMETRY, 1,
                                  &CommanderNode::odometryCB, this);

    thrust_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>
                    (pyramid_msgs::default_topics::COMMAND_THRUST, 1);
}

CommanderNode::~CommanderNode(){ }

void CommanderNode::paramsReconfig(pyramid_control::SlidingModeControllerConfig &config,
                                            uint32_t level)
{
    sliding_mode_controller_.system_parameters_.Lambda_(0, 0) = config.lambda_1;
    sliding_mode_controller_.system_parameters_.Lambda_(1, 1) = config.lambda_1;
    sliding_mode_controller_.system_parameters_.Lambda_(2, 2) = config.lambda_1;
    sliding_mode_controller_.system_parameters_.Lambda_(3, 3) = config.lambda_1;
    sliding_mode_controller_.system_parameters_.Lambda_(4, 4) = config.lambda_1;
    sliding_mode_controller_.system_parameters_.Lambda_(5, 5) = config.lambda_1;

    sliding_mode_controller_.system_parameters_.K_(0, 0) = config.K_1;
    sliding_mode_controller_.system_parameters_.K_(1, 1) = config.K_2;
    sliding_mode_controller_.system_parameters_.K_(2, 2) = config.K_3;
    sliding_mode_controller_.system_parameters_.K_(3, 3) = config.K_4;
    sliding_mode_controller_.system_parameters_.K_(4, 4) = config.K_5;
    sliding_mode_controller_.system_parameters_.K_(5, 5) = config.K_6;
}

void CommanderNode::trajectoryCB(
                            const trajectory_msgs::MultiDOFJointTrajectoryPtr& trajectory_msg)
{
    ROS_INFO_ONCE("Recieved first Desired Trajectory. SMC START!");

    pyramid_msgs::EigenMultiDOFJointTrajectory trajectory;
    pyramid_msgs::eigenMultiDOFJointTrajectoryFromMsg(trajectory_msg, &trajectory);

    sliding_mode_controller_.setTrajectory(trajectory);
}

void CommanderNode::odometryCB(const nav_msgs::OdometryPtr& odometry_msg)
{
    ROS_INFO_ONCE("Recieved first odometry msg.");

    pyramid_msgs::EigenOdometry odometry;
    pyramid_msgs::eigenOdometryFromMsg(odometry_msg, &odometry);

    sliding_mode_controller_.setOdometry(odometry);
    sliding_mode_controller_.updateModelConfig();
    sliding_mode_controller_.calcThrust();

    sendThrust();

    Eigen::VectorXd wrench = sliding_mode_controller_.getWrench();
    Eigen::MatrixXd jacobian = sliding_mode_controller_.getJacobian();
    Eigen::Matrix3d rotMatrix = sliding_mode_controller_.getRotMatrix();
    Eigen::Matrix3d toOmega = sliding_mode_controller_.getToOmega();
    ROS_INFO_ONCE("GOOD1");
    actuator_controller_.wrenchDistribution(wrench, jacobian, rotMatrix, toOmega);
    actuator_controller_.optimize();
    ROS_INFO_ONCE("GOOD2");

}

void CommanderNode::sendThrust()
{
    geometry_msgs::WrenchStamped thrust_msg;

    pyramid_msgs::EigenThrust thrust = sliding_mode_controller_.getThrust();
    thrust_msg.header.stamp = ros::Time::now();
    pyramid_msgs::eigenThrustToMsg(thrust, thrust_msg);

    thrust_pub_.publish(thrust_msg);
}

} // namespace pyramid_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "commander_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  pyramid_control::CommanderNode commander_node(nh, private_nh);

  ros::spin();

  return 0;
}
