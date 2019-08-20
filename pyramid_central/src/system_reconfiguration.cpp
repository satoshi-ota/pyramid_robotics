#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pyramid_central/SystemCommanderConfig.h>

void callback(pyramid_central::SystemCommanderConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f %f %f %f %f",
            config.P_x, config.P_y, config.P_z,
            config.P_roll, config.P_pitch, config.P_yaw,
            config.D_x, config.D_y, config.D_z,
            config.D_roll, config.D_pitch, config.D_yaw);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "reconfigure");

  dynamic_reconfigure::Server<pyramid_central::SystemCommanderConfig> server;
  dynamic_reconfigure::Server<pyramid_central::SystemCommanderConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
