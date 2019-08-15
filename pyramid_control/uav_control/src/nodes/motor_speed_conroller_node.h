#ifndef MOTOR_SPEED_CONTROLLER_NODE_H
#define MOTOR_SPEED_CONTROLLER_NODE_H

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mav_msgs/Actuators.h>
#include <pyramid_msgs/pyramid_eigen_msgs.h>

namespace motor_speed_control
{

class MotorSpeedControllerNode
{
public:
    MotorSpeedControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~MotorSpeedControllerNode();

    void InitializeParams();

private: //member data
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //class
    MotorSpeedController motor_speed_controller_;

    // subscribers
    ros::Subscriber thrust_cmd_sub_;

    ros::Publisher motor_velocity_reference_pub_;

private: //member function
    void ThrustCommandCB(const geometry_msgs::WrenchStampedPtr& thrust_msg);
};

} //motor_speed_control

#endif //MOTOR_SPEED_CONTROLLER_NODE_H
