#include "motor_speed_controller_node.h"

namespace motor_speed_control
{

MotorSpeedControllerNode::MotorSpeedControllerNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    InitializeParams();

    thrust_cmd_sub_ = nh_.subscribe("thrust_command", 1,
                            &MotorSpeedControllerNode::ThrustCommandCB, this);

    motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>("motor_speed", 1);
}

MotorSpeedControllerNode::~MotorSpeedControllerNode(){ }

void MotorSpeedControllerNode::InitializeParams()
{
    GetVehicleParameters(private_nh_, &motor_speed_controller_.vehicle_parameters_);
    motor_speed_controller_.InitializeParameters();
}

void MotorSpeedControllerNode::ThrustCommandCB(const geometry_msgs::WrenchStampedPtr& thrust_msg)
{
    ROS_INFO_ONCE("MotorSpeedConroller got first thrust msg.");

    //conversion
    pyramid_msgs::EigenWrenchStamped desired_thrust;
    eigenThrustFromMsg(thrust_msg, &desired_thrust);
    motor_speed_controller_.SetThrustMsg(desired_thrust);

    //calculate motor speed
    Eigen::VectorXd ref_rotor_velocities;
    motor_speed_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

    //write motor speed
    actuator_msg->angular_velocities.clear();
    for (int i = 0; i < ref_rotor_velocities.size(); i++)
      actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
    actuator_msg->header.stamp = thrust_msg->header.stamp;

    //publish
    motor_velocity_reference_pub_.publish(actuator_msg);
}

} //motor_speed_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_speed_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  motor_speed_control::MotorSpeedControllerNode motor_speed_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
