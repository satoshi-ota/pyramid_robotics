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
    observer_ = new Observer(&system_parameters_);

    GetSystemParameters(private_nh_, &system_parameters_);

    actuator_controller_->InitializeParameters();

    srv_ = boost::make_shared
            <dynamic_reconfigure::Server<pyramid_control::SystemParametersConfig>>(private_nh);
    dynamic_reconfigure::Server<pyramid_control::SystemParametersConfig>::CallbackType cb
        = boost::bind(&CommanderNode::paramsReconfig, this, _1, _2);
    srv_->setCallback(cb);

    trajectory_sub_ = nh_.subscribe(pyramid_msgs::default_topics::COMMAND_TRAJECTORY, 1,
                                    &CommanderNode::trajectoryCB, this);

    odometry_sub_ = nh_.subscribe(pyramid_msgs::default_topics::FEEDBACK_ODOMETRY, 1,
                                  &CommanderNode::odometryCB, this);

    motor_speed_ref_pub_ = nh_.advertise<mav_msgs::Actuators>
                                    (pyramid_msgs::default_topics::COMMAND_ACTUATORS, 1);

    tension_ref_pub_ = nh_.advertise<pyramid_msgs::Tensions>
                             (pyramid_msgs::default_topics::COMMAND_TENSIONS, 1);

    motor_speed_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>
                  (pyramid_msgs::default_topics::MARKER_THURUST, 1);

    tension_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>
                          (pyramid_msgs::default_topics::MARKER_TENSION, 1);

    winch_pub_ = nh_.advertise<pyramid_msgs::Positions>
                 (pyramid_msgs::default_topics::COMMAND_ANCHOR_POS, 1);

    // thrust_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>
    //                 (pyramid_msgs::default_topics::COMMAND_THRUST, 1);
}

CommanderNode::~CommanderNode(){ }

void CommanderNode::paramsReconfig(pyramid_control::SystemParametersConfig &config,
                                            uint32_t level)
{
    system_parameters_.reconfig(config);
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

    observer_->estimateDisturbance(wrench);

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

    motor_speed_ref_pub_.publish(actuator_msg);

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

    motor_speed_marker_pub_.publish(marker_array);
}

void CommanderNode::sendTension()
{
    geometry_msgs::Vector3 tension;

    Eigen::VectorXd ref_tensions = actuator_controller_->getTension();
    pyramid_msgs::TensionsPtr tension_msg(new pyramid_msgs::Tensions);

    tension_msg->tensions.clear();
    for(int i = 0; i < ref_tensions.size(); i++)
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

    tension_ref_pub_.publish(tension_msg);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(ref_tensions.size());

    geometry_msgs::Vector3 arrow;
    arrow.x = 0.02;
    arrow.y = 0.04;
    arrow.z = 0.1;

    for(int i = 0; i < ref_tensions.size(); i++)
    {
        geometry_msgs::Point linear_start;
        linear_start.x = system_parameters_.tether_configuration_.pseudo_tethers[i].world_pos.x();
        linear_start.y = system_parameters_.tether_configuration_.pseudo_tethers[i].world_pos.y();
        linear_start.z = system_parameters_.tether_configuration_.pseudo_tethers[i].world_pos.z();

        geometry_msgs::Point linear_end;
        linear_end.x = linear_start.x + ref_tensions[i] *
            system_parameters_.tether_configuration_.pseudo_tethers[i].direction.x();
        linear_end.y = linear_start.y + ref_tensions[i] *
            system_parameters_.tether_configuration_.pseudo_tethers[i].direction.y();
        linear_end.z = linear_start.z + ref_tensions[i] *
            system_parameters_.tether_configuration_.pseudo_tethers[i].direction.z();

        marker_array.markers[i].header.frame_id = "world";
        marker_array.markers[i].header.stamp = ros::Time::now();
        marker_array.markers[i].ns = "pelican";
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

        marker_array.markers[i].color.r = 0.0f;
        marker_array.markers[i].color.g = 1.0f;
        marker_array.markers[i].color.b = 0.0f;
        marker_array.markers[i].color.a = 0.5f;
    }

    tension_marker_pub_.publish(marker_array);
}

void CommanderNode::sendWinchPos()
{
    geometry_msgs::Vector3 position;
    pyramid_msgs::PositionsPtr position_msg(new pyramid_msgs::Positions);

    position_msg->positions.clear();

    for (int i = 0; i < system_parameters_.n_anchor_; i++)
    {
        position.x = system_parameters_.tether_configuration_.pseudo_tethers[2*i].anchor_pos.x();
        position.y = system_parameters_.tether_configuration_.pseudo_tethers[2*i].anchor_pos.y();
        position.z = system_parameters_.tether_configuration_.pseudo_tethers[2*i].anchor_pos.z();

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
