#ifndef PYRAMID_CENTRAL_TRAJECTORY_COMMANDER_NODE_H
#define PYRAMID_CENTRAL_TRAJECTORY_COMMANDER_NODE_H

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <pyramid_msgs/conversions.h>
#include <pyramid_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>

#include "pyramid_central/common.h"
#include "pyramid_central/system_commander.h"
#include "pyramid_central/system_reconfiguration.h"
#include "pyramid_central/SystemCommanderConfig.h"

namespace system_commander
{

class TrajectoryCommanderNode
{
public:
    TrajectoryCommanderNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~TrajectoryCommanderNode();

    void SetTrajectoryCB(pyramid_central::SystemCommanderConfig &config, uint32_t level);

private:
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    boost::shared_ptr<dynamic_reconfigure::Server<pyramid_central::SystemCommanderConfig>> srv_;

    SystemReconfigure system_reconfigure_;

    ros::Publisher trajectory_pub_;
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
};

} //namespace system_commander

#endif //PYRAMID_CENTRAL_TRAJECTORY_COMMANDER_NODE_H
