#ifndef PYRAMID_CENTRAL_PARAMETERS_H
#define PYRAMID_CENTRAL_PARAMETERS_H

namespace system_commander {
// Default values for the Asctec Firefly rotor configuration.
static constexpr double kDefaultRotor0Angle = 0.52359877559;
static constexpr double kDefaultRotor1Angle = 1.57079632679;
static constexpr double kDefaultRotor2Angle = 2.61799387799;
static constexpr double kDefaultRotor3Angle = -2.61799387799;
static constexpr double kDefaultRotor4Angle = -1.57079632679;
static constexpr double kDefaultRotor5Angle = -0.52359877559;

// Default vehicle parameters for Asctec Firefly.
static constexpr double kDefaultMass = 1.56779;
static constexpr double kDefaultArmLength = 0.215;
static constexpr double kDefaultInertiaXx = 0.0347563;
static constexpr double kDefaultInertiaYy = 0.0458929;
static constexpr double kDefaultInertiaZz = 0.0977;
static constexpr double kDefaultRotorForceConstant = 8.54858e-6;
static constexpr double kDefaultRotorMomentConstant = 1.6e-2;

// Default tether parameters
const unsigned int DefaultTetherNum = 4;
//mounting_posisons
const Eigen::Vector3d DefaultTether0MountPos = Eigen::Vector3d( 0.21,   0.0, 0.0);
const Eigen::Vector3d DefaultTether1MountPos = Eigen::Vector3d(  0.0, -0.21, 0.0);
const Eigen::Vector3d DefaultTether2MountPos = Eigen::Vector3d(-0.21,   0.0, 0.0);
const Eigen::Vector3d DefaultTether3MountPos = Eigen::Vector3d(  0.0,  0.21, 0.0);

//directions
const Eigen::Vector3d DefaultTether0Direction = Eigen::Vector3d( 1.0,  0.0, 0.0);
const Eigen::Vector3d DefaultTether1Direction = Eigen::Vector3d( 0.0, -1.0, 0.0);
const Eigen::Vector3d DefaultTether2Direction = Eigen::Vector3d(-1.0,  0.0, 0.0);
const Eigen::Vector3d DefaultTether3Direction = Eigen::Vector3d( 0.0,  1.0, 0.0);

//anchor_positions
const Eigen::Vector3d DefaultAnchor0Pos = Eigen::Vector3d( 5.21,   0.0, 0.0);
const Eigen::Vector3d DefaultAnchor1Pos = Eigen::Vector3d(  0.0, -5.21, 0.0);
const Eigen::Vector3d DefaultAnchor2Pos = Eigen::Vector3d(-5.21,   0.0, 0.0);
const Eigen::Vector3d DefaultAnchor3Pos = Eigen::Vector3d(  0.0,  5.21, 0.0);

const Eigen::Matrix<double, 6, 6> DefaultGainP = Eigen::MatrixXd::Identity(6, 6);
const Eigen::Matrix<double, 6, 6> DefaultGainD = Eigen::MatrixXd::Identity(6, 6);

// Default physics parameters.
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

struct Tether
{
    Tether()
        :mounting_pos(Eigen::Vector3d::Zero()),
         direction(Eigen::Vector3d::Zero()),
         tension(0.0),
         anchor_position(Eigen::Vector3d::Zero()){ }

    Tether(Eigen::Vector3d _mounting_pos,
           Eigen::Vector3d _anchor_position)
        :mounting_pos(_mounting_pos),
         direction(Eigen::Vector3d::Zero()),
         tension(0.0),
         anchor_position(_anchor_position){ }

    Tether(Eigen::Vector3d _mounting_pos,
           Eigen::Vector3d _direction,
           double _tension,
           Eigen::Vector3d _anchor_position)
        :mounting_pos(_mounting_pos),
         direction(_direction),
         tension(_tension),
         anchor_position(_anchor_position){ }

    Eigen::Vector3d mounting_pos; //body-fixed frame
    Eigen::Vector3d direction; //global frame
    double tension;
    Eigen::Vector3d anchor_position; //global_frame
};

struct TetherConfiguration
{
    TetherConfiguration()
    {
        tethers.push_back(
            Tether(DefaultTether0MountPos, DefaultTether0Direction, 0.0, DefaultAnchor0Pos));
        tethers.push_back(
            Tether(DefaultTether1MountPos, DefaultTether1Direction, 0.0, DefaultAnchor1Pos));
        tethers.push_back(
            Tether(DefaultTether2MountPos, DefaultTether2Direction, 0.0, DefaultAnchor2Pos));
        tethers.push_back(
            Tether(DefaultTether3MountPos, DefaultTether3Direction, 0.0, DefaultAnchor3Pos));
    }
    std::vector<Tether> tethers;
};

class SystemParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SystemParameters()
      : mass_(kDefaultMass),
        gravity_(kDefaultGravity),
        inertia_(Eigen::Vector3d(kDefaultInertiaXx, kDefaultInertiaYy,
                                 kDefaultInertiaZz).asDiagonal()),
        n_tether_(DefaultTetherNum),
        K_p_(DefaultGainP),
        K_d_(DefaultGainD){ }
  double mass_;
  const double gravity_;
  Eigen::Matrix3d inertia_;
  int n_tether_;

  //PID control parameters
  Eigen::Matrix<double, 6, 6> K_d_;
  Eigen::Matrix<double, 6, 6> K_p_;

  RotorConfiguration rotor_configuration_;
  TetherConfiguration tether_configuration_;
};

}

#endif //PYRAMID_CENTRAL_PARAMETERS_H
