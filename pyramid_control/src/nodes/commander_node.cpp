#include "commander_node.h"

namespace pyramid_control
{

CommanderNode::CommanderNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    sliding_mode_controller_ = new SlidingModeController(&system_parameters_);
    actuator_controller_ = new ActuatorController(&system_parameters_);

    GetSystemParameters(private_nh_, &system_parameters_);

    actuator_controller_->InitializeParameters();

    srv_ = boost::make_shared
            <dynamic_reconfigure::Server<pyramid_control::SlidingModeControllerConfig>>(private_nh);
    dynamic_reconfigure::Server<pyramid_control::SlidingModeControllerConfig>::CallbackType cb
        = boost::bind(&CommanderNode::paramsReconfig, this, _1, _2);
    srv_->setCallback(cb);

    trajectory_sub_ = nh_.subscribe(pyramid_msgs::default_topics::COMMAND_TRAJECTORY, 1,
                                    &CommanderNode::trajectoryCB, this);

    odometry_sub_ = nh_.subscribe(pyramid_msgs::default_topics::FEEDBACK_ODOMETRY, 1,
                                  &CommanderNode::odometryCB, this);

    motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>
                                    (pyramid_msgs::default_topics::COMMAND_ACTUATORS, 1);

    tension_reference_pub_ = nh_.advertise<pyramid_msgs::Tensions>
                             (pyramid_msgs::default_topics::COMMAND_TENSIONS, 1);

    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>
                  (pyramid_msgs::default_topics::MARKER_THURUST, 1);

    winch_pub_ = nh_.advertise<pyramid_msgs::Positions>
                 (pyramid_msgs::default_topics::COMMAND_ANCHOR_POS, 1);

    // thrust_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>
    //                 (pyramid_msgs::default_topics::COMMAND_THRUST, 1);
}

CommanderNode::~CommanderNode(){ }

void CommanderNode::paramsReconfig(pyramid_control::SlidingModeControllerConfig &config,
                                            uint32_t level)
{
    system_parameters_.Lambda_(0, 0) = config.lambda_1;
    system_parameters_.Lambda_(1, 1) = config.lambda_1;
    system_parameters_.Lambda_(2, 2) = config.lambda_1;
    system_parameters_.Lambda_(3, 3) = config.lambda_1;
    system_parameters_.Lambda_(4, 4) = config.lambda_1;
    system_parameters_.Lambda_(5, 5) = config.lambda_1;

    system_parameters_.K_(0, 0) = config.K_1;
    system_parameters_.K_(1, 1) = config.K_2;
    system_parameters_.K_(2, 2) = config.K_3;
    system_parameters_.K_(3, 3) = config.K_4;
    system_parameters_.K_(4, 4) = config.K_5;
    system_parameters_.K_(5, 5) = config.K_6;
    //
    Eigen::Vector3d position;

    winch_pos_.clear();

    position.x() = config.winch_1_x;
    position.y() = config.winch_1_y;
    position.z() = config.winch_1_z;
    winch_pos_.push_back(position);

    position.x() = config.winch_2_x;
    position.y() = config.winch_2_y;
    position.z() = config.winch_2_z;
    winch_pos_.push_back(position);

    position.x() = config.winch_3_x;
    position.y() = config.winch_3_y;
    position.z() = config.winch_3_z;
    winch_pos_.push_back(position);

    position.x() = config.winch_4_x;
    position.y() = config.winch_4_y;
    position.z() = config.winch_4_z;
    winch_pos_.push_back(position);
    //
    // sliding_mode_controller_.system_parameters_.tether_configuration_.setAnchorPos(winch_pos_[0], 0);
    // sliding_mode_controller_.system_parameters_.tether_configuration_.setAnchorPos(winch_pos_[2], 1);
    // sliding_mode_controller_.system_parameters_.tether_configuration_.setAnchorPos(winch_pos_[1], 2);
    // sliding_mode_controller_.system_parameters_.tether_configuration_.setAnchorPos(winch_pos_[3], 3);
    // sliding_mode_controller_.system_parameters_.tether_configuration_.setAnchorPos(winch_pos_[2], 4);
    // sliding_mode_controller_.system_parameters_.tether_configuration_.setAnchorPos(winch_pos_[0], 5);
    // sliding_mode_controller_.system_parameters_.tether_configuration_.setAnchorPos(winch_pos_[3], 6);
    // sliding_mode_controller_.system_parameters_.tether_configuration_.setAnchorPos(winch_pos_[1], 7);

    // sliding_mode_controller_.system_parameters_.tether_configuration_.pseudo_tethers[0].anchor_pos
    //     = winch_pos_[0];
    // sliding_mode_controller_.system_parameters_.tether_configuration_.pseudo_tethers[1].anchor_pos
    //     = winch_pos_[2];
    // sliding_mode_controller_.system_parameters_.tether_configuration_.pseudo_tethers[2].anchor_pos
    //     = winch_pos_[1];
    // sliding_mode_controller_.system_parameters_.tether_configuration_.pseudo_tethers[3].anchor_pos
    //     = winch_pos_[3];
    // sliding_mode_controller_.system_parameters_.tether_configuration_.pseudo_tethers[4].anchor_pos
    //     = winch_pos_[2];
    // sliding_mode_controller_.system_parameters_.tether_configuration_.pseudo_tethers[5].anchor_pos
    //     = winch_pos_[0];
    // sliding_mode_controller_.system_parameters_.tether_configuration_.pseudo_tethers[6].anchor_pos
    //     = winch_pos_[3];
    // sliding_mode_controller_.system_parameters_.tether_configuration_.pseudo_tethers[7].anchor_pos
    //     = winch_pos_[1];

}

void CommanderNode::trajectoryCB(
                            const trajectory_msgs::MultiDOFJointTrajectoryPtr& trajectory_msg)
{
    ROS_INFO_ONCE("Recieved first Desired Trajectory. SMC START!");

    pyramid_msgs::EigenMultiDOFJointTrajectory trajectory;
    pyramid_msgs::eigenMultiDOFJointTrajectoryFromMsg(trajectory_msg, &trajectory);

    sliding_mode_controller_->setTrajectory(trajectory);
}

void CommanderNode::odometryCB(const nav_msgs::OdometryPtr& odometry_msg)
{
    ROS_INFO_ONCE("Recieved first odometry msg.");

    pyramid_msgs::EigenOdometry odometry;
    pyramid_msgs::eigenOdometryFromMsg(odometry_msg, &odometry);

    system_parameters_.setOdom(odometry);
    sliding_mode_controller_->updateModelConfig();
    sliding_mode_controller_->calcThrust();

    // sendThrust();

    Eigen::VectorXd wrench = sliding_mode_controller_->getWrench();

    actuator_controller_->wrenchDistribution(wrench);
    actuator_controller_->optimize();

    sendRotorSpeed();
    sendTension();
    sendWinchPos();
}

void CommanderNode::sendRotorSpeed()
{
    Eigen::VectorXd ref_rotor_velocities = actuator_controller_->getMotorSpeed();

    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

    actuator_msg->angular_velocities.clear();
    for(int i = 0; i < ref_rotor_velocities.size(); i++)
        actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
    actuator_msg->header.stamp = ros::Time::now();

    motor_velocity_reference_pub_.publish(actuator_msg);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(4);

    for(int i = 0; i < ref_rotor_velocities.size(); i++)
    {
        geometry_msgs::Point linear_start;
        linear_start.x = 0;
        linear_start.y = 0;
        linear_start.z = 0;

        geometry_msgs::Point linear_end;
        linear_end.x = 0;
        linear_end.y = 0;
        linear_end.z = ref_rotor_velocities[i] / 1000;

        geometry_msgs::Vector3 arrow;
        arrow.x = 0.02;
        arrow.y = 0.04;
        arrow.z = 0.1;

        std::string rotor_num = std::to_string(i);
        std::string frame_id = "/pelican/rotor_" + rotor_num;

        marker_array.markers[i].header.frame_id = frame_id;
        marker_array.markers[i].header.stamp = ros::Time::now();
        marker_array.markers[i].ns = "/pelican";
        marker_array.markers[i].id = i;
        marker_array.markers[i].lifetime = ros::Duration();

        marker_array.markers[i].type = visualization_msgs::Marker::ARROW;
        marker_array.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array.markers[i].scale = arrow;

        marker_array.markers[i].points.resize(2);
        marker_array.markers[i].points[0] = linear_start;
        marker_array.markers[i].points[1] = linear_end;

        marker_array.markers[i].pose.orientation.x = 0.0;
        marker_array.markers[i].pose.orientation.y = 0.0;
        marker_array.markers[i].pose.orientation.z = 0.0;
        marker_array.markers[i].pose.orientation.w = 1.0;

        marker_array.markers[i].color.r = 1.0f;
        marker_array.markers[i].color.g = 0.0f;
        marker_array.markers[i].color.b = 0.0f;
        marker_array.markers[i].color.a = 0.5f;
    }

    marker_pub_.publish(marker_array);
}

void CommanderNode::sendTension()
{
    geometry_msgs::Vector3 tension;

    Eigen::VectorXd ref_tensions = actuator_controller_->getTension();
    pyramid_msgs::TensionsPtr tension_msg(new pyramid_msgs::Tensions);

    tension_msg->tensions.clear();
    for (int i = 0; i < ref_tensions.size(); i++)
    {
        tension.x = ref_tensions[i] *
            system_parameters_.tether_configuration_.pseudo_tethers[i].direction.x();
        tension.y = ref_tensions[i] *
            system_parameters_.tether_configuration_.pseudo_tethers[i].direction.y();
        tension.z = ref_tensions[i] *
            system_parameters_.tether_configuration_.pseudo_tethers[i].direction.z();

        tension_msg->tensions.push_back(tension);
    }

    tension_msg->header.stamp = ros::Time::now();

    tension_reference_pub_.publish(tension_msg);
}

void CommanderNode::sendWinchPos()
{
    geometry_msgs::Vector3 position;
    pyramid_msgs::PositionsPtr position_msg(new pyramid_msgs::Positions);

    position_msg->positions.clear();

    for (int i = 0; i < winch_pos_.size(); i++)
    {
        position.x = winch_pos_[i].x();
        position.y = winch_pos_[i].y();
        position.z = winch_pos_[i].z();

        position_msg->positions.push_back(position);
    }
    position_msg->header.stamp = ros::Time::now();

    winch_pub_.publish(position_msg);
}

// void CommanderNode::sendThrust()
// {
//     geometry_msgs::WrenchStamped thrust_msg;
//
//     pyramid_msgs::EigenThrust thrust = sliding_mode_controller_.getThrust();
//     thrust_msg.header.stamp = ros::Time::now();
//     pyramid_msgs::eigenThrustToMsg(thrust, thrust_msg);
//
//     thrust_pub_.publish(thrust_msg);
// }

} // namespace pyramid_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "commander_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  pyramid_control::CommanderNode commander_node(nh, private_nh);

  ros::spin();

  return 0;
}
