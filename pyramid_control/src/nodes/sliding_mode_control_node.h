#ifndef PYRAMID_CONTROL_SLIDING_MODE_CONTROL_NODE_H
#define PYRAMID_CONTROL_SLIDING_MODE_CONTROL_NODE_H

#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <pyramid_msgs/conversions.h>
#include <pyramid_msgs/default_topics.h>
#include <geometry_msgs/WrenchStamped.h>

#include "pyramid_control/common_central.h"
#include "pyramid_control/sliding_mode_controller.h"
#include "pyramid_control/system_reconfiguration.h"
#include "pyramid_control/SlidingModeControllerConfig.h"

namespace pyramid_control
{

class SlidingModeControlNode
{
public:
    SlidingModeControlNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~SlidingModeControlNode();

    // void InitializeParams();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    boost::shared_ptr<dynamic_reconfigure::Server<pyramid_control::SlidingModeControllerConfig>> srv_;

    SlidingModeController sliding_mode_controller_;
    SystemReconfigure system_reconfigure_;

    ros::Subscriber trajectory_sub_;
    ros::Subscriber odometry_sub_;
    ros::Publisher thrust_pub_;

    void reconfigureCB(pyramid_control::SlidingModeControllerConfig &config, uint32_t level);
    void trajectoryCB(const trajectory_msgs::MultiDOFJointTrajectoryPtr& trajectory_msg);
    void odometryCB(const nav_msgs::OdometryPtr& odometry_msg);
    void sendThrust();
};

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_SLIDING_MODE_CONTROL_NODE_H
