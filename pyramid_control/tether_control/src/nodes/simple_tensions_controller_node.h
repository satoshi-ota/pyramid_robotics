#ifndef SIMPLE_TENSIONS_CONTROLLER_NODE_H
#define SIMPLE_TENSIONS_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>

namespace tensions_control
{

class SimpleTensionsControllerNode
{
public:
    SimpleTensionsControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~SimpleTensionsControllerNode();

    inline unsigned int n_tether(){return n_tether_;}
    inline unsigned int hz(){return hz_;}

    void InitializeParams();
    void sendTensions(Eigen::Vector4d& td);

private:
    //general
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //tensions
    double t0_, t1_, t2_, t3_;

    //model params
    int n_tether_;

    //subscribe gazebo feelback
    ros::Subscriber tether_states_sub_, end_effector_state_sub_;

    //publisher to tether tensions
    ros::Publisher tensions_pub_;
    sensor_msgs::JointState tensions_msg;
    int hz_;
};
} //namespace tensions_control

#endif //SIMPLE_TENSIONS_CONTROLLER_NODE_H
