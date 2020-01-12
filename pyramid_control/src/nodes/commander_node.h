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
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>

#include "pyramid_control/observer.h"
#include "pyramid_control/actuator_controller.h"
#include "pyramid_control/sliding_mode_controller.h"
#include "pyramid_control/SystemParametersConfig.h"

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

    boost::shared_ptr<dynamic_reconfigure::Server<pyramid_control::SystemParametersConfig>> srv_;

    SlidingModeController *sliding_mode_controller_;
    ActuatorController *actuator_controller_;
    Observer *observer_;

    SystemParameters system_parameters_;

    ros::Subscriber trajectory_sub_;
    ros::Subscriber odometry_sub_;
    // ros::Subscriber imu_sub_;
    ros::Publisher motor_speed_ref_pub_;
    ros::Publisher motor_speed_marker_pub_;
    ros::Publisher gz_tension_ref_pub_;
    ros::Publisher tension_ref_pub_;
    ros::Publisher tension_marker_pub_;
    ros::Publisher winch_pub_;
    ros::Publisher disturbance_pub_;
    ros::Publisher thrust_pub_;

    bool enable_observer_;

    void paramsReconfig(pyramid_control::SystemParametersConfig &config, uint32_t level);
    void trajectoryCB(const trajectory_msgs::MultiDOFJointTrajectoryPtr& trajectory_msg);
    // void imuCB(const sensor_msgs::ImuPtr& Imu_msg);
    void odometryCB(const nav_msgs::OdometryPtr& odometry_msg);

    void sendRotorSpeed(const Eigen::VectorXd& motor_speed);
    void sendTension(const Eigen::VectorXd& ref_tensions);
    void sendWinchPos();
    void sendEstDisturbance(const Eigen::VectorXd& disturbance);
    void sendThrust(const Eigen::VectorXd& wrench);
};

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_COMMANDER_NODE_H
