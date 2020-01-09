#ifndef PYRAMID_EVALUATION_DISTURBANCE_GEN_NODE_H
#define PYRAMID_EVALUATION_DISTURBANCE_GEN_NODE_H

#include <Eigen/Core>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pyramid_msgs/conversions.h>
#include <pyramid_msgs/default_topics.h>
#include <geometry_msgs/WrenchStamped.h>
#include <dynamic_reconfigure/server.h>

#include "pyramid_evaluation/DisturbanceGeneratorConfig.h"

namespace pyramid_evaluation
{

class DisturbanceGenNode
{
public:
    DisturbanceGenNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~DisturbanceGenNode();

    void sendDisturbance();
    void disturbanceReconfig(pyramid_evaluation::DisturbanceGeneratorConfig &config, uint32_t level);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    boost::shared_ptr<dynamic_reconfigure::Server<pyramid_evaluation::DisturbanceGeneratorConfig>> srv_;

    Eigen::Vector3d force_;
    Eigen::Vector3d torque_;

    ros::Publisher disturbance_pub_;

    std::string disturbance_mode_;

    bool enableDisturbance_;
    bool takeoff_;
    double semiMinorAxis_, semiMajorAxis_;
};

} //namespace pyramid_evaluation

#endif //PYRAMID_EVALUATION_DISTURBANCE_GEN_NODE_H
