#ifndef PYRAMID_CENTRAL_CONFIGURATIONS_H
#define PYRAMID_CENTRAL_CONFIGURATIONS_H

#include <ros/ros.h>

#include "pyramid_central/parameters.h"

namespace system_commander
{

template<typename T> inline void GetRosParameter(const ros::NodeHandle& nh,
                                                 const std::string& key,
                                                 const T& default_value,
                                                       T* value)
{
    ROS_ASSERT(value != nullptr);
    bool have_parameter = nh.getParam(key, *value);
    if (!have_parameter)
    {
      ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                      << "/" << key << ", setting to default: " << default_value);
      *value = default_value;
    }
}

inline void GetRotorConfiguration(const ros::NodeHandle& nh,
                                  RotorConfiguration* rotor_configuration)
{
    std::map<std::string, double> single_rotor;
    std::string rotor_configuration_string = "rotor_configuration/";
    unsigned int i = 0;
    while (nh.getParam(rotor_configuration_string + std::to_string(i), single_rotor))
    {
        if (i == 0) {
            rotor_configuration->rotors.clear();
        }
        Rotor rotor;
        nh.getParam(rotor_configuration_string + std::to_string(i) + "/angle",
            rotor.angle);
        nh.getParam(rotor_configuration_string + std::to_string(i) + "/arm_length",
            rotor.arm_length);
        nh.getParam(rotor_configuration_string + std::to_string(i) + "/rotor_force_constant",
            rotor.rotor_force_constant);
        nh.getParam(rotor_configuration_string + std::to_string(i) + "/rotor_moment_constant",
            rotor.rotor_moment_constant);
        nh.getParam(rotor_configuration_string + std::to_string(i) + "/direction",
            rotor.direction);
        rotor_configuration->rotors.push_back(rotor);
        ++i;
    }
}

inline void GetTetherConfiguration(const ros::NodeHandle& nh,
                                  TetherConfiguration* tether_configuration)
{
    std::map<std::string, double> single_tether;
    std::string tether_configuration_string = "tether_configuration/";
    unsigned int i = 0;
    while (nh.getParam(tether_configuration_string + std::to_string(i), single_tether))
    {
        if (i == 0) {
            tether_configuration->tethers.clear();
        }
        Eigen::Vector3d mounting_pos;
        Eigen::Vector3d anchor_position;

        nh.getParam(tether_configuration_string + std::to_string(i) + "/mounting_pos/x",
            mounting_pos.x());
        nh.getParam(tether_configuration_string + std::to_string(i) + "/mounting_pos/y",
            mounting_pos.y());
        nh.getParam(tether_configuration_string + std::to_string(i) + "/mounting_pos/z",
            mounting_pos.z());
        nh.getParam(tether_configuration_string + std::to_string(i) + "/anchor_pos/x",
            anchor_position.x());
        nh.getParam(tether_configuration_string + std::to_string(i) + "/anchor_pos/y",
            anchor_position.y());
        nh.getParam(tether_configuration_string + std::to_string(i) + "/anchor_pos/y",
            anchor_position.z());

        tether_configuration->tethers.push_back(Tether(mounting_pos, anchor_position));
        ++i;
    }
}

inline void GetSystemParameters(const ros::NodeHandle& nh, SystemParameters* system_parameters)
{
    //uav parameters
    GetRosParameter(nh, "mass",
        system_parameters->mass_,
        &system_parameters->mass_);
    GetRosParameter(nh, "inertia/xx",
        system_parameters->inertia_(0, 0),
        &system_parameters->inertia_(0, 0));
    GetRosParameter(nh, "inertia/xy",
        system_parameters->inertia_(0, 1),
        &system_parameters->inertia_(0, 1));
        system_parameters->inertia_(1, 0) = system_parameters->inertia_(0, 1);
    GetRosParameter(nh, "inertia/xz",
        system_parameters->inertia_(0, 2),
        &system_parameters->inertia_(0, 2));
        system_parameters->inertia_(2, 0) = system_parameters->inertia_(0, 2);
    GetRosParameter(nh, "inertia/yy",
        system_parameters->inertia_(1, 1),
        &system_parameters->inertia_(1, 1));
    GetRosParameter(nh, "inertia/yz",
        system_parameters->inertia_(1, 2),
        &system_parameters->inertia_(1, 2));
        system_parameters->inertia_(2, 1) = system_parameters->inertia_(1, 2);
    GetRosParameter(nh, "inertia/zz",
        system_parameters->inertia_(2, 2),
        &system_parameters->inertia_(2, 2));

    //tether number
    GetRosParameter(nh, "n_tether",
        system_parameters->n_tether_,
        &system_parameters->n_tether_);

    //PID parameters D
    GetRosParameter(nh, "gainD/x",
        system_parameters->K_d_(0, 0),
        &system_parameters->K_d_(0, 0));
    GetRosParameter(nh, "gainD/y",
        system_parameters->K_d_(1, 1),
        &system_parameters->K_d_(1, 1));
    GetRosParameter(nh, "gainD/z",
        system_parameters->K_d_(2, 2),
        &system_parameters->K_d_(2, 2));
    GetRosParameter(nh, "gainD/roll",
        system_parameters->K_d_(3, 3),
        &system_parameters->K_d_(3, 3));
    GetRosParameter(nh, "gainD/pitch",
        system_parameters->K_d_(4, 4),
        &system_parameters->K_d_(4, 4));
    GetRosParameter(nh, "gainD/yaw",
        system_parameters->K_d_(5, 5),
        &system_parameters->K_d_(5, 5));

    //PID parameters P
    GetRosParameter(nh, "gainP/x",
        system_parameters->K_p_(0, 0),
        &system_parameters->K_p_(0, 0));
    GetRosParameter(nh, "gainP/y",
        system_parameters->K_p_(1, 1),
        &system_parameters->K_p_(1, 1));
    GetRosParameter(nh, "gainP/z",
        system_parameters->K_p_(2, 2),
        &system_parameters->K_p_(2, 2));
    GetRosParameter(nh, "gainP/roll",
        system_parameters->K_p_(3, 3),
        &system_parameters->K_p_(3, 3));
    GetRosParameter(nh, "gainP/pitch",
        system_parameters->K_p_(4, 4),
        &system_parameters->K_p_(4, 4));
    GetRosParameter(nh, "gainP/yaw",
        system_parameters->K_p_(5, 5),
        &system_parameters->K_p_(5, 5));

    //Sliding Mode Controller parameters Lambda
    GetRosParameter(nh, "lambda_1",
        system_parameters->Lambda_(0, 0),
        &system_parameters->Lambda_(0, 0));
    GetRosParameter(nh, "lambda_2",
        system_parameters->Lambda_(1, 1),
        &system_parameters->Lambda_(1, 1));
    GetRosParameter(nh, "lambda_3",
        system_parameters->Lambda_(2, 2),
        &system_parameters->Lambda_(2, 2));
    GetRosParameter(nh, "lambda_4",
        system_parameters->Lambda_(3, 3),
        &system_parameters->Lambda_(3, 3));
    GetRosParameter(nh, "lambda_5",
        system_parameters->Lambda_(4, 4),
        &system_parameters->Lambda_(4, 4));
    GetRosParameter(nh, "lambda_6",
        system_parameters->Lambda_(5, 5),
        &system_parameters->Lambda_(5, 5));

    //Sliding Mode Controller parameters K
    GetRosParameter(nh, "K_1",
        system_parameters->K_(0, 0),
        &system_parameters->K_(0, 0));
    GetRosParameter(nh, "K_2",
        system_parameters->K_(1, 1),
        &system_parameters->K_(1, 1));
    GetRosParameter(nh, "K_3",
        system_parameters->K_(2, 2),
        &system_parameters->K_(2, 2));
    GetRosParameter(nh, "K_4",
        system_parameters->K_(3, 3),
        &system_parameters->K_(3, 3));
    GetRosParameter(nh, "K_5",
        system_parameters->K_(4, 4),
        &system_parameters->K_(4, 4));
    GetRosParameter(nh, "K_6",
        system_parameters->K_(5, 5),
        &system_parameters->K_(5, 5));

    GetRotorConfiguration(nh, &system_parameters->rotor_configuration_);
}

} //namespace system_commander

#endif //PYRAMID_CENTRAL_CONFIGURATIONS_H
