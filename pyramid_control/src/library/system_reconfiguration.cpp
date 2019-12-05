#include "pyramid_control/system_reconfiguration.h"

namespace pyramid_control
{

void SystemReconfigure::smcReconfig(
                            pyramid_control::SlidingModeControllerConfig& config,
                            SystemParameters* system_parameters)
{
    // ROS_INFO("Reconfigure Request: Lambda[%f, %f, %f, %f, %f, %f] Gain_K[%f, %f, %f, %f, %f, %f]",
    //          config.lambda_1, config.lambda_2, config.lambda_3,
    //          config.lambda_4, config.lambda_5, config.lambda_6,
    //          config.K_1, config.K_2, config.K_3, config.K_4, config.K_5, config.K_6);

    system_parameters->Lambda_(0, 0) = config.lambda_1;
    system_parameters->Lambda_(1, 1) = config.lambda_1;
    system_parameters->Lambda_(2, 2) = config.lambda_1;
    system_parameters->Lambda_(3, 3) = config.lambda_1;
    system_parameters->Lambda_(4, 4) = config.lambda_1;
    system_parameters->Lambda_(5, 5) = config.lambda_1;

    system_parameters->K_(0, 0) = config.K_1;
    system_parameters->K_(1, 1) = config.K_2;
    system_parameters->K_(2, 2) = config.K_3;
    system_parameters->K_(3, 3) = config.K_4;
    system_parameters->K_(4, 4) = config.K_5;
    system_parameters->K_(5, 5) = config.K_6;
}

void SystemReconfigure::trajectoryReconfig(pyramid_control::TrajectoryGeneratorConfig& config)
{
    // ROS_INFO("Reconfigure Request: Desired position[%f, %f, %f] Desired Euler Angle[%f, %f, %f]",
    //          config.x, config.y, config.z, config.roll, config.pitch, config.yaw);

    pos_.x() = config.x;
    pos_.y() = config.y;
    pos_.z() = config.z;

    att_.x() = config.roll;
    att_.y() = config.pitch;
    att_.z() = config.yaw;
}

} //namespace pyramid_control
