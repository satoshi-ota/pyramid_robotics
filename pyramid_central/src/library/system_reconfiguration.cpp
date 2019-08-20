#include "pyramid_central/system_reconfiguration.h"

namespace system_commander
{

void SystemReconfigure::reconfig(pyramid_central::SystemCommanderConfig& config,
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

} //namespace system_commander
