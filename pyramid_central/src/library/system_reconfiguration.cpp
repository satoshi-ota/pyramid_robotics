#include "pyramid_central/system_reconfiguration.h"

namespace system_commander
{

void SystemReconfigure::PIDControllerReconfig(pyramid_central::SystemCommanderConfig& config,
                                              SystemParameters* system_parameters)
{
    ROS_INFO("Reconfigure Request: P[%f, %f, %f, %f, %f, %f] D[%f, %f, %f, %f, %f, %f]",
             config.P_x, config.P_y, config.P_z, config.P_roll, config.P_pitch, config.P_yaw,
             config.D_x, config.D_y, config.D_z, config.D_roll, config.D_pitch, config.D_yaw);

    system_parameters->K_d_(0, 0) = config.D_x;
    system_parameters->K_d_(1, 1) = config.D_y;
    system_parameters->K_d_(2, 2) = config.D_z;
    system_parameters->K_d_(3, 3) = config.D_roll;
    system_parameters->K_d_(4, 4) = config.D_pitch;
    system_parameters->K_d_(5, 5) = config.D_yaw;

    system_parameters->K_p_(0, 0) = config.P_x;
    system_parameters->K_p_(1, 1) = config.P_y;
    system_parameters->K_p_(2, 2) = config.P_z;
    system_parameters->K_p_(3, 3) = config.P_roll;
    system_parameters->K_p_(4, 4) = config.P_pitch;
    system_parameters->K_p_(5, 5) = config.P_yaw;
}

void SystemReconfigure::SlidingModeControllerReconfig(
                            pyramid_central::SlidingModeControllerConfig& config,
                            SystemParameters* system_parameters)
{
    ROS_INFO("Reconfigure Request: Lambda[%f, %f, %f, %f, %f, %f] Gain_K[%f, %f, %f, %f, %f, %f]",
             config.lambda_1, config.lambda_2, config.lambda_3,
             config.lambda_4, config.lambda_5, config.lambda_6,
             config.K_1, config.K_2, config.K_3, config.K_4, config.K_5, config.K_6);

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

void SystemReconfigure::TrajectoryReconfig(pyramid_central::TrajectoryGeneratorConfig& config)
{
    ROS_INFO("Reconfigure Request: Desired position[%f, %f, %f] Desired Attitude[0.0, 0.0, %f]",
             config.x, config.y, config.z, config.yaw);

    desired_position_.x() = config.x;
    desired_position_.y() = config.y;
    desired_position_.z() = config.z;

    desired_yaw_ = config.yaw;
}

} //namespace system_commander
