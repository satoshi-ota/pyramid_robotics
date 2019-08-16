#include "simple_tensions_controller_node.h"

namespace tensions_control
{

SimpleTensionsControllerNode::SimpleTensionsControllerNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh){
    //get model params
    InitializeParams();

    //publisher to tensions_msg
    tensions_pub_ = nh_.advertise<sensor_msgs::JointState>
                        (pyramid_msgs::default_topics::COMMAND_TENSIONS, 1);
}

SimpleTensionsControllerNode::~SimpleTensionsControllerNode(){ }

void SimpleTensionsControllerNode::InitializeParams()
{
    // Overwrite defaults if set as node parameters.
    //get desired tensions
    private_nh_.getParam("tension_0", t0_);
    private_nh_.getParam("tension_1", t1_);
    private_nh_.getParam("tension_2", t2_);
    private_nh_.getParam("tension_3", t3_);

    //get tether number
    n_tether_ = 4;
    private_nh_.getParam("n_tether", n_tether_);

    //get tensions_pub frequency
    private_nh_.getParam("hz", hz_);

    //initialize tensions_msg
    char tether_name[256];
    for(unsigned int i=0;i<n_tether_;++i)
    {
        sprintf(tether_name, "ugv2tether_joint_prismatic_%i", i);
        tensions_msg.name.push_back(std::string(tether_name));
    }
    tensions_msg.effort.resize(n_tether_);
}

void SimpleTensionsControllerNode::sendTensions(Eigen::Vector4d& td)
{
    //write tether tensions
    tensions_msg.header.stamp = ros::Time::now();
    for(unsigned int i=0;i<n_tether_;++i)
        tensions_msg.effort[i] = td(i, 0);

    tensions_pub_.publish(tensions_msg);
}

} //namespace tensions_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_tensions_controller_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    tensions_control::SimpleTensionsControllerNode simple_tensions_conrtoller_node(nh, private_nh);

    Eigen::Vector4d desired_tensions(0.0, 1.0, 1.0, 0.0);

    unsigned int hz = simple_tensions_conrtoller_node.hz();
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        simple_tensions_conrtoller_node.sendTensions(desired_tensions);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
