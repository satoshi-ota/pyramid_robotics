#include <ros/ros.h>

namespace tensions_control
{

} //namespace tensions_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_tensions_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rotors_control::LeePositionControllerNode lee_position_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
