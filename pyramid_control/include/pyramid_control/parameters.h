#ifndef PYRAMID_CONTROL_PARAMETERS_H
#define PYRAMID_CONTROL_PARAMETERS_H

#include "pyramid_control/SystemParametersConfig.h"

namespace pyramid_control
{
static constexpr double kDefaultRotor0Angle = 0.52359877559;
static constexpr double kDefaultRotor1Angle = 1.57079632679;
static constexpr double kDefaultRotor2Angle = 2.61799387799;
static constexpr double kDefaultRotor3Angle = -2.61799387799;
static constexpr double kDefaultRotor4Angle = -1.57079632679;
static constexpr double kDefaultRotor5Angle = -0.52359877559;

static constexpr double kDefaultMass = 1.56779;
static constexpr double kDefaultArmLength = 0.215;
static constexpr double kDefaultInertiaXx = 0.0347563;
static constexpr double kDefaultInertiaYy = 0.0458929;
static constexpr double kDefaultInertiaZz = 0.0977;
static constexpr double kDefaultRotorForceConstant = 8.54858e-6;
static constexpr double kDefaultRotorMomentConstant = 1.6e-2;

const Eigen::Vector3d kDefaultTether0AttachPos = Eigen::Vector3d( 0.4, -0.4,  0.15);
const Eigen::Vector3d kDefaultTether1AttachPos = Eigen::Vector3d( 0.4, -0.4, -0.15);
const Eigen::Vector3d kDefaultTether2AttachPos = Eigen::Vector3d( 0.4,  0.4,  0.15);
const Eigen::Vector3d kDefaultTether3AttachPos = Eigen::Vector3d( 0.4,  0.4, -0.15);
const Eigen::Vector3d kDefaultTether4AttachPos = Eigen::Vector3d(-0.4,  0.4,  0.15);
const Eigen::Vector3d kDefaultTether5AttachPos = Eigen::Vector3d(-0.4,  0.4, -0.15);
const Eigen::Vector3d kDefaultTether6AttachPos = Eigen::Vector3d(-0.4, -0.4,  0.15);
const Eigen::Vector3d kDefaultTether7AttachPos = Eigen::Vector3d(-0.4, -0.4, -0.15);

const Eigen::Vector3d kDefaultAnchor0Pos = Eigen::Vector3d( 5.0,  5.0, 0.0);
const Eigen::Vector3d kDefaultAnchor1Pos = Eigen::Vector3d(-5.0,  5.0, 0.0);
const Eigen::Vector3d kDefaultAnchor2Pos = Eigen::Vector3d(-5.0, -5.0, 0.0);
const Eigen::Vector3d kDefaultAnchor3Pos = Eigen::Vector3d( 5.0, -5.0, 0.0);

static constexpr double kDefaultGravity = 9.81;

struct Rotor {
    Rotor()
    : angle(0.0),
    arm_length(kDefaultArmLength),
    rotor_force_constant(kDefaultRotorForceConstant),
    rotor_moment_constant(kDefaultRotorMomentConstant),
    direction(1) {}
    Rotor(double _angle, double _arm_length,
        double _rotor_force_constant, double _rotor_moment_constant,
        int _direction)
        : angle(_angle),
        arm_length(_arm_length),
        rotor_force_constant(_rotor_force_constant),
        rotor_moment_constant(_rotor_moment_constant),
        direction(_direction) {}
        double angle;
        double arm_length;
        double rotor_force_constant;
        double rotor_moment_constant;
        int direction;
};

struct RotorConfiguration {
  RotorConfiguration() {
    // Rotor configuration of Asctec Firefly.
    rotors.push_back(
      Rotor(kDefaultRotor0Angle, kDefaultArmLength, kDefaultRotorForceConstant,
            kDefaultRotorMomentConstant, 1));
    rotors.push_back(
      Rotor(kDefaultRotor1Angle, kDefaultArmLength, kDefaultRotorForceConstant,
            kDefaultRotorMomentConstant, -1));
    rotors.push_back(
      Rotor(kDefaultRotor2Angle, kDefaultArmLength, kDefaultRotorForceConstant,
            kDefaultRotorMomentConstant, 1));
    rotors.push_back(
      Rotor(kDefaultRotor3Angle, kDefaultArmLength, kDefaultRotorForceConstant,
            kDefaultRotorMomentConstant, -1));
    rotors.push_back(
      Rotor(kDefaultRotor4Angle, kDefaultArmLength, kDefaultRotorForceConstant,
            kDefaultRotorMomentConstant, 1));
    rotors.push_back(
      Rotor(kDefaultRotor5Angle, kDefaultArmLength, kDefaultRotorForceConstant,
            kDefaultRotorMomentConstant, -1));
  }
  std::vector<Rotor> rotors;
};

struct PseudoTether
{
    PseudoTether()
        :attach_pos(Eigen::Vector3d::Zero()),
         direction(Eigen::Vector3d::Zero()),
         anchor_pos(Eigen::Vector3d::Zero()),
         world_pos(Eigen::Vector3d::Zero()){ }

    PseudoTether(const Eigen::Vector3d& _attach_pos,
                 const Eigen::Vector3d& _anchor_pos)
        :attach_pos(_attach_pos),
         direction(Eigen::Vector3d::Zero()),
         anchor_pos(_anchor_pos),
         world_pos(Eigen::Vector3d::Zero()){ }

    void update(const pyramid_msgs::EigenOdometry& odometry)
    {
        world_pos = odometry.position + odometry.orientation.toRotationMatrix() * attach_pos;
        direction = anchor_pos - world_pos;
        direction = direction.normalized();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double max_tension;
    Eigen::Vector3d attach_pos;
    Eigen::Vector3d direction;
    Eigen::Vector3d anchor_pos;
    Eigen::Vector3d world_pos;
};

struct TetherConfiguration
{
    TetherConfiguration()
    {
        pseudo_tethers.push_back(PseudoTether(kDefaultTether0AttachPos, kDefaultAnchor0Pos));
        pseudo_tethers.push_back(PseudoTether(kDefaultTether1AttachPos, kDefaultAnchor2Pos));
        pseudo_tethers.push_back(PseudoTether(kDefaultTether2AttachPos, kDefaultAnchor1Pos));
        pseudo_tethers.push_back(PseudoTether(kDefaultTether3AttachPos, kDefaultAnchor3Pos));
        pseudo_tethers.push_back(PseudoTether(kDefaultTether4AttachPos, kDefaultAnchor2Pos));
        pseudo_tethers.push_back(PseudoTether(kDefaultTether5AttachPos, kDefaultAnchor0Pos));
        pseudo_tethers.push_back(PseudoTether(kDefaultTether6AttachPos, kDefaultAnchor3Pos));
        pseudo_tethers.push_back(PseudoTether(kDefaultTether7AttachPos, kDefaultAnchor1Pos));
    }
    std::vector<PseudoTether> pseudo_tethers;

    void setAnchorPos(const double& x, const double& y, const double& z, int i)
    {
        pseudo_tethers[i].anchor_pos.x() = x;
        pseudo_tethers[i].anchor_pos.y() = y;
        pseudo_tethers[i].anchor_pos.z() = z;
    }
};

class SystemParameters
{
public:
    SystemParameters()
        :mass_(kDefaultMass),
         gravity_(kDefaultGravity),
         inertia_(Eigen::Vector3d(kDefaultInertiaXx,
                                  kDefaultInertiaYy,
                                  kDefaultInertiaZz).asDiagonal()),
         Lambda_(Eigen::MatrixXd::Identity(6, 6)),
         K_(Eigen::MatrixXd::Identity(6, 6)),
         rotMatrix_(Eigen::Matrix3d::Zero()),
         toOmega_(Eigen::Matrix3d::Zero()),
         toOmega_dot_(Eigen::Matrix3d::Zero()),
         massMatrix_(Eigen::MatrixXd::Zero(6, 6)),
         coriolisMatrix_(Eigen::MatrixXd::Zero(6, 6)){ }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RotorConfiguration rotor_configuration_;
    TetherConfiguration tether_configuration_;

    pyramid_msgs::EigenOdometry odometry_;

    Eigen::Vector3d momentum_;
    Eigen::Vector3d omegaXmomentum_;

    Eigen::Matrix3d rotMatrix_;
    Eigen::Matrix3d toOmega_;
    Eigen::Matrix3d toOmega_dot_;
    Eigen::Matrix3d skewMatrix_;

    Eigen::MatrixXd massMatrix_;
    Eigen::MatrixXd coriolisMatrix_;
    Eigen::MatrixXd jacobian_;

    Eigen::Matrix<double, 6, 6> Lambda_;
    Eigen::Matrix<double, 6, 6> K_;

    double mass_;
    double gravity_;
    Eigen::Matrix3d inertia_;

    inline void reconfig(pyramid_control::SystemParametersConfig& config)
    {
        Lambda_(0, 0) = config.lambda_1;
        Lambda_(1, 1) = config.lambda_2;
        Lambda_(2, 2) = config.lambda_3;
        Lambda_(3, 3) = config.lambda_4;
        Lambda_(4, 4) = config.lambda_5;
        Lambda_(5, 5) = config.lambda_6;

        K_(0, 0) = config.K_1;
        K_(1, 1) = config.K_2;
        K_(2, 2) = config.K_3;
        K_(3, 3) = config.K_4;
        K_(4, 4) = config.K_5;
        K_(5, 5) = config.K_6;

        tether_configuration_.setAnchorPos(config.anchor1_x, config.anchor1_y, config.anchor1_z, 0);
        tether_configuration_.setAnchorPos(config.anchor3_x, config.anchor3_y, config.anchor3_z, 1);
        tether_configuration_.setAnchorPos(config.anchor2_x, config.anchor2_y, config.anchor2_z, 2);
        tether_configuration_.setAnchorPos(config.anchor4_x, config.anchor4_y, config.anchor4_z, 3);
        tether_configuration_.setAnchorPos(config.anchor3_x, config.anchor3_y, config.anchor3_z, 4);
        tether_configuration_.setAnchorPos(config.anchor1_x, config.anchor1_y, config.anchor1_z, 5);
        tether_configuration_.setAnchorPos(config.anchor4_x, config.anchor4_y, config.anchor4_z, 6);
        tether_configuration_.setAnchorPos(config.anchor2_x, config.anchor2_y, config.anchor2_z, 7);
    }

    inline void setOdom(const pyramid_msgs::EigenOdometry& odometry)
    {
        odometry_ = odometry;
        rotMatrix_ = odometry_.orientation.toRotationMatrix();
    }

    inline void update()
    {
        for(PseudoTether& pseudo_tether : tether_configuration_.pseudo_tethers)
        {
            pseudo_tether.update(odometry_);
        }
    }

    inline void calcskewMatrix()
    {
        Eigen::Vector3d vector = odometry_.angular_velocity;

        skewMatrix_ <<          0,-vector.z(),  vector.y(),
                       vector.z(),          0, -vector.x(),
                      -vector.y(), vector.x(),           0;
    }

    inline void calcMomentum()
    {
        momentum_ =  inertia_ * odometry_.angular_velocity;
    }

    inline void calcOmegaxMomentum()
    {
        omegaXmomentum_ = skewMatrix_ * momentum_;
    }

    inline void calcToOmage()
    {
        Eigen::Vector3d rpy;
        pyramid_msgs::getEulerAnglesFromQuaternion(odometry_.orientation, &rpy);

        toOmega_ << 1,  0,           -sin(rpy(1)),
                    0,  cos(rpy(0)),  cos(rpy(1))*sin(rpy(0)),
                    0, -sin(rpy(0)),  cos(rpy(1))*cos(rpy(0));

        toOmega_dot_ << 0,  0,           -cos(rpy(1)),
                        0, -sin(rpy(0)), -sin(rpy(1))*sin(rpy(0)) + cos(rpy(1))*cos(rpy(0)),
                        0, -cos(rpy(0)), -sin(rpy(1))*cos(rpy(0)) - cos(rpy(1))*sin(rpy(0));
    }

    inline void calcMassMatrix()
    {
        Eigen::Matrix3d kI = Eigen::Matrix3d::Identity();

        massMatrix_.setZero();
        massMatrix_.topLeftCorner(3, 3) = mass_ * kI.array();
        massMatrix_.bottomRightCorner(3, 3) = inertia_ * toOmega_;
    }

    inline void calcCoriolisMatrix()
    {
        coriolisMatrix_.setZero();
        coriolisMatrix_.bottomRightCorner(3, 3)
            = inertia_ * toOmega_dot_ + skewMatrix_ * inertia_ * toOmega_;
    }

    inline void calcJacobian()
    {
        int tether_num = tether_configuration_.pseudo_tethers.size();

        jacobian_.resize(tether_num, 6);
        jacobian_.setZero();

        Eigen::MatrixXd J;
        J.resize(6, tether_num);
        J.setZero();

        unsigned int i = 0;
        for (const PseudoTether& pseudo_tether : tether_configuration_.pseudo_tethers)
        {
            J.block<3, 1>(0, i) = pseudo_tether.direction;

            Eigen::Vector3d ub = rotMatrix_.inverse() * pseudo_tether.direction;
            J.block<3, 1>(3, i) = pseudo_tether.attach_pos.cross(ub);

            ++i;
        }
        jacobian_ = J.transpose();
    }
};

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_PARAMETERS_H
