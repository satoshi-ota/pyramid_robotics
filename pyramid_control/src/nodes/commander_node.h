#ifndef PYRAMID_CONTROL_COMMANDER_NODE_H
#define PYRAMID_CONTROL_COMMANDER_NODE_H

#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/Actuators.h>
#include <pyramid_msgs/Tensions.h>
#include <pyramid_msgs/Positions.h>
#include <pyramid_msgs/conversions.h>
#include <pyramid_msgs/default_topics.h>
#include <geometry_msgs/WrenchStamped.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>

#include "pyramid_control/actuator_controller.h"
#include "pyramid_control/sliding_mode_controller.h"
#include "pyramid_control/SlidingModeControllerConfig.h"

namespace pyramid_control
{

class CommanderNode
{
public:
    CommanderNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~CommanderNode();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    boost::shared_ptr<dynamic_reconfigure::Server<pyramid_control::SlidingModeControllerConfig>> srv_;

    SlidingModeController *sliding_mode_controller_;
    ActuatorController *actuator_controller_;

    SystemParameters system_parameters_;

    ros::Subscriber trajectory_sub_;
    ros::Subscriber odometry_sub_;
    ros::Publisher motor_velocity_reference_pub_;
    ros::Publisher tension_reference_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher winch_pub_;

    std::vector<Eigen::Vector3d> winch_pos_;

    // ros::Publisher thrust_pub_;

    void paramsReconfig(pyramid_control::SlidingModeControllerConfig &config, uint32_t level);
    void trajectoryCB(const trajectory_msgs::MultiDOFJointTrajectoryPtr& trajectory_msg);
    void odometryCB(const nav_msgs::OdometryPtr& odometry_msg);

    void sendRotorSpeed();
    void sendTension();
    void sendWinchPos();
    // void sendThrust();
};

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_COMMANDER_NODE_H
