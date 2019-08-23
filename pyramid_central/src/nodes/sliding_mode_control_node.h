#ifndef PYRAMID_CENTRAL_SYSTEM_COMMANDER_NODE_H
#define PYRAMID_CENTRAL_SYSTEM_COMMANDER_NODE_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <pyramid_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>

#include "pyramid_central/common.h"
#include "pyramid_central/sliding_mode_controller.h"
#include "pyramid_central/system_reconfiguration.h"
#include "pyramid_central/SystemCommanderConfig.h"

namespace system_commander
{

class SlidingModeControlNode
{
public:
    SlidingModeControlNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~SlidingModeControlNode();

    void InitializeParams();
    void ControllerReconfigureCB(pyramid_central::SystemCommanderConfig &config, uint32_t level);
    void sendThrust();

private: //member data
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Time begin_;

    boost::shared_ptr<dynamic_reconfigure::Server<pyramid_central::SystemCommanderConfig>> srv_;

    //topic
    geometry_msgs::WrenchStamped thrust_msg;

    //class
    SlidingModeController sliding_mode_controller_;
    SystemReconfigure system_reconfigure_;

    //subscriber
    ros::Subscriber trajectory_sub_;
    ros::Subscriber odometry_sub_;

    //publisher
    ros::Publisher thrust_pub_;

private: //member function
    void DesiredTrajectoryCB(const trajectory_msgs::MultiDOFJointTrajectoryPtr& trajectory_msg);
    void FeedbackOdometryCB(const nav_msgs::OdometryPtr& odometry_msg);
};

} //namespace system_commander

#endif //PYRAMID_CENTRAL_SYSTEM_COMMANDER_NODE_H
