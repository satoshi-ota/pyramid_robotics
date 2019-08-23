#ifndef PYRAMID_MSGS_DEFAULT_TOPICS_H
#define PYRAMID_MSGS_DEFAULT_TOPICS_H

namespace pyramid_msgs {
namespace default_topics {
//command
static constexpr char COMMAND_TENSIONS[] = "/command/tension";
static constexpr char COMMAND_THRUST[] = "/command/thrust";
static constexpr char COMMAND_ACTUATORS[] = "command/motor_speed";
static constexpr char COMMAND_TRAJECTORY[] = "command/trajectory";
static constexpr char COMMAND_POSE[] = "command/pose";

//state
static constexpr char STATE_TETHER[] = "state/tether";
static constexpr char STATE_ODOMETRY[] = "state/odometry";
//feedback
static constexpr char FEEDBACK_IMU[] = "feedback/imu";
static constexpr char FEEDBACK_ODOMETRY[] = "feedback/odometry";
}  // end namespace default_topics
}  // end namespace mav_msgs

#endif // PYRAMID_MSGS_DEFAULT_TOPICS_H
