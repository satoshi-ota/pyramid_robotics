#ifndef PYRAMID_CONTROL_TRAJECTORY_COMMANDER_NODE_H
#define PYRAMID_CONTROL_TRAJECTORY_COMMANDER_NODE_H

#include <Eigen/Core>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pyramid_msgs/conversions.h>
#include <pyramid_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <dynamic_reconfigure/server.h>

#include "pyramid_control/TrajectoryGeneratorConfig.h"

namespace pyramid_control
{

class TrajectoryCommanderNode
{
public:
    TrajectoryCommanderNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~TrajectoryCommanderNode();

    void trajectoryReconfig(pyramid_control::TrajectoryGeneratorConfig &config, uint32_t level);
    void targetPosAtt();
    void ellipticOrbit();
    void sendOrbit();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    boost::shared_ptr<dynamic_reconfigure::Server<pyramid_control::TrajectoryGeneratorConfig>> srv_;

    Eigen::Vector3d pos_;
    Eigen::Vector3d att_;

    ros::Publisher trajectory_pub_;
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

    std::string trajectory_mode_;
    double semiMinorAxis_, semiMajorAxis_;
};

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_TRAJECTORY_COMMANDER_NODE_H
