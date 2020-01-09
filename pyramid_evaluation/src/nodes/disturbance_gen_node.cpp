#include "disturbance_gen_node.h"

namespace pyramid_evaluation
{

DisturbanceGenNode::DisturbanceGenNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh)
{
    disturbance_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>
                        (pyramid_msgs::default_topics::COMMAND_DISTURBANCE, 10);

    srv_ = boost::make_shared
            <dynamic_reconfigure::Server<pyramid_evaluation::DisturbanceGeneratorConfig>>(private_nh);
    dynamic_reconfigure::Server<pyramid_evaluation::DisturbanceGeneratorConfig>::CallbackType cb
        = boost::bind(&DisturbanceGenNode::disturbanceReconfig, this, _1, _2);
    srv_->setCallback(cb);
}

DisturbanceGenNode::~DisturbanceGenNode(){ }

void DisturbanceGenNode::disturbanceReconfig(pyramid_evaluation::DisturbanceGeneratorConfig &config,
                                             uint32_t level)
{
    disturbance_mode_ = config.disturbance_modes;

    enableDisturbance_ = config.enable_disturbance;

    force_.x() = config.force_x;
    force_.y() = config.force_y;
    force_.z() = config.force_z;

    torque_.x() = config.torque_x;
    torque_.y() = config.torque_y;
    torque_.z() = config.torque_z;

    semiMinorAxis_ = config.semiMinorAxis;
    semiMajorAxis_ = config.semiMajorAxis;
}

void DisturbanceGenNode::sendDisturbance()
{
    geometry_msgs::WrenchStamped disturbance_msg;

    disturbance_msg.header.stamp = ros::Time::now();
    disturbance_msg.wrench.force.x = force_.x();
    disturbance_msg.wrench.force.y = force_.y();
    disturbance_msg.wrench.force.z = force_.z();
    disturbance_msg.wrench.torque.x = torque_.x();
    disturbance_msg.wrench.torque.y = torque_.y();
    disturbance_msg.wrench.torque.z = torque_.z();

    disturbance_pub_.publish(disturbance_msg);
}

} //namespace pyramid_evaluation

int main(int argc, char** argv) {
    ros::init(argc, argv, "disturbance_gen_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    pyramid_evaluation::DisturbanceGenNode disturbance_gen_node(nh, private_nh);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        disturbance_gen_node.sendDisturbance();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
