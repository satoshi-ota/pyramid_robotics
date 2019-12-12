#ifndef PYRAMID_CONTROL_PARAMETERS_H
#define PYRAMID_CONTROL_PARAMETERS_H

namespace pyramid_control
{
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

const int kDefaultDoF = 6;
const int kDefaultTetherNum = 8;

//attach_posisons
const Eigen::Vector3d kDefaultTether0AttachPos = Eigen::Vector3d( 0.4, -0.4,  0.15);
const Eigen::Vector3d kDefaultTether1AttachPos = Eigen::Vector3d( 0.4, -0.4, -0.15);
const Eigen::Vector3d kDefaultTether2AttachPos = Eigen::Vector3d( 0.4,  0.4,  0.15);
const Eigen::Vector3d kDefaultTether3AttachPos = Eigen::Vector3d( 0.4,  0.4, -0.15);
const Eigen::Vector3d kDefaultTether4AttachPos = Eigen::Vector3d(-0.4,  0.4,  0.15);
const Eigen::Vector3d kDefaultTether5AttachPos = Eigen::Vector3d(-0.4,  0.4, -0.15);
const Eigen::Vector3d kDefaultTether6AttachPos = Eigen::Vector3d(-0.4, -0.4,  0.15);
const Eigen::Vector3d kDefaultTether7AttachPos = Eigen::Vector3d(-0.4, -0.4, -0.15);

//anchor_positions
const Eigen::Vector3d kDefaultAnchor0Pos = Eigen::Vector3d( 5.0,  5.0, 0.0);
const Eigen::Vector3d kDefaultAnchor1Pos = Eigen::Vector3d(-5.0,  5.0, 0.0);
const Eigen::Vector3d kDefaultAnchor2Pos = Eigen::Vector3d(-5.0, -5.0, 0.0);
const Eigen::Vector3d kDefaultAnchor3Pos = Eigen::Vector3d( 5.0, -5.0, 0.0);

//Sliding mode controller
const Eigen::Matrix<double, 6, 6> kDefaultLambda = Eigen::MatrixXd::Identity(6, 6);
const Eigen::Matrix<double, 6, 6> kDefaultGainK = Eigen::MatrixXd::Identity(6, 6);

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

struct PseudoTether
{
    PseudoTether()
        :attach_pos(Eigen::Vector3d::Zero()),
         direction(Eigen::Vector3d::Zero()),
         anchor_pos(Eigen::Vector3d::Zero()),
         world_pos(Eigen::Vector3d::Zero()),
         tension(0.0){ }

    PseudoTether(const Eigen::Vector3d& _attach_pos,
                 const Eigen::Vector3d& _anchor_pos)
        :attach_pos(_attach_pos),
         direction(Eigen::Vector3d::Zero()),
         anchor_pos(_anchor_pos),
         world_pos(Eigen::Vector3d::Zero()),
         tension(0.0){ }

    void update(const pyramid_msgs::EigenOdometry& odometry)
    {
        world_pos = odometry.position + odometry.orientation.toRotationMatrix() * attach_pos;
        direction = anchor_pos - world_pos;
        direction = direction.normalized();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double tension;
    double maxTension, minTension;
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
         dof_(kDefaultDoF),
         n_tether_(kDefaultTetherNum),
         Lambda_(kDefaultLambda),
         K_(kDefaultGainK){ }

    RotorConfiguration rotor_configuration_;
    TetherConfiguration tether_configuration_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double mass_;
    double gravity_;
    Eigen::Matrix3d inertia_;
    int dof_;
    int n_tether_;

    Eigen::Matrix<double, 6, 6> Lambda_;
    Eigen::Matrix<double, 6, 6> K_;
};

} //namespace pyramid_control

#endif //PYRAMID_CONTROL_PARAMETERS_H
