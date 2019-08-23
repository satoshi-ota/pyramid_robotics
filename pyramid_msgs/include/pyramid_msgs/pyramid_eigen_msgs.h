#ifndef PYRAMID_MSGS_PYRAMID_EIGEN_MSGS_H
#define PYRAMID_MSGS_PYRAMID_EIGEN_MSGS_H

#include <Eigen/Eigen>

#include "pyramid_msgs/common.h"

namespace pyramid_msgs
{

struct EigenWrenchStamped
{
    EigenWrenchStamped()
        :timestamp_ns(-1),
         force(Eigen::Vector3d::Zero()),
         torque(Eigen::Vector3d::Zero()){ }

    EigenWrenchStamped(const Eigen::Vector3d& _force,
                       const Eigen::Vector3d& _torque)
        :force(_force),
         torque(_torque){ }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int64_t timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
    Eigen::Vector3d force;
    Eigen::Vector3d torque;

    inline double getZforce() const {return force.z();}
    inline Eigen::Vector3d getTorque() const {return torque;}
};

struct EigenThrust
{
    EigenThrust()
        :timestamp_ns(-1),
         force(Eigen::Vector3d::Zero()),
         torque(Eigen::Vector3d::Zero()){ }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int64_t timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
    Eigen::Vector3d force;
    Eigen::Vector3d torque;
};

struct EigenMultiDOFJointTrajectory
{
    EigenMultiDOFJointTrajectory()
        :timestamp_ns(-1),
         time_from_start_ns(0),
         position(Eigen::Vector3d::Zero()),
         velocity(Eigen::Vector3d::Zero()),
         acceleration(Eigen::Vector3d::Zero()),
         orientation(Eigen::Quaterniond::Identity()),
         angular_velocity(Eigen::Vector3d::Zero()),
         angular_acceleration(Eigen::Vector3d::Zero()){ }

    EigenMultiDOFJointTrajectory(int64_t _time_from_start_ns,
                                 const Eigen::Vector3d& _position,
                                 const Eigen::Vector3d& _velocity,
                                 const Eigen::Vector3d& _acceleration,
                                 const Eigen::Quaterniond& _orientation,
                                 const Eigen::Vector3d& _angular_velocity,
                                 const Eigen::Vector3d& _angular_acceleration)
        :time_from_start_ns(_time_from_start_ns),
         position(_position),
         velocity(_velocity),
         acceleration(_acceleration),
         orientation(_orientation),
         angular_velocity(_angular_velocity),
         angular_acceleration(_angular_acceleration){ }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int64_t timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
    int64_t time_from_start_ns;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d angular_acceleration;

    inline Eigen::VectorXd getAcc() const
    {
        Eigen::VectorXd acc = Eigen::VectorXd::Zero(6);
        acc.block<3, 1>(0, 0) = acceleration;
        acc.block<3, 1>(3, 0) = angular_acceleration;
        return acc;
    }

    inline Eigen::VectorXd getVel() const
    {
        Eigen::VectorXd vel = Eigen::VectorXd::Zero(6);
        vel.block<3, 1>(0, 0) = velocity;
        vel.block<3, 1>(3, 0) = angular_velocity;
        return vel;
    }

    inline Eigen::VectorXd getPosAtt() const
    {
        Eigen::Vector3d rpy;
        getEulerAnglesFromQuaternion(orientation, &rpy);
        Eigen::VectorXd posatt = Eigen::VectorXd::Zero(6);
        posatt.block<3, 1>(0, 0) = position;
        posatt.block<3, 1>(3, 0) = rpy;
        return posatt;
    }
};

struct EigenTrajectoryPoint
{
    typedef std::vector<EigenTrajectoryPoint, Eigen::aligned_allocator<EigenTrajectoryPoint>> Vector;
    EigenTrajectoryPoint()
        :timestamp_ns(-1),
         time_from_start_ns(0),
         position(Eigen::Vector3d::Zero()),
         velocity(Eigen::Vector3d::Zero()),
         acceleration(Eigen::Vector3d::Zero()),
         jerk(Eigen::Vector3d::Zero()),
         snap(Eigen::Vector3d::Zero()),
         orientation(Eigen::Quaterniond::Identity()),
         angular_velocity(Eigen::Vector3d::Zero()),
         angular_acceleration(Eigen::Vector3d::Zero()) {}

    EigenTrajectoryPoint(int64_t _time_from_start_ns,
                         const Eigen::Vector3d& _position,
                         const Eigen::Vector3d& _velocity,
                         const Eigen::Vector3d& _acceleration,
                         const Eigen::Vector3d& _jerk,
                         const Eigen::Vector3d& _snap,
                         const Eigen::Quaterniond& _orientation,
                         const Eigen::Vector3d& _angular_velocity,
                         const Eigen::Vector3d& _angular_acceleration)
        :time_from_start_ns(_time_from_start_ns),
         position(_position),
         velocity(_velocity),
         acceleration(_acceleration),
         jerk(_jerk),
         snap(_snap),
         orientation(_orientation),
         angular_velocity(_angular_velocity),
         angular_acceleration(_angular_acceleration) {}

    EigenTrajectoryPoint(int64_t _time_from_start_ns,
                         const Eigen::Vector3d& _position,
                         const Eigen::Vector3d& _velocity,
                         const Eigen::Vector3d& _acceleration,
                         const Eigen::Vector3d& _jerk,
                         const Eigen::Vector3d& _snap,
                         const Eigen::Quaterniond& _orientation,
                         const Eigen::Vector3d& _angular_velocity)
        :EigenTrajectoryPoint(_time_from_start_ns, _position, _velocity,
                              _acceleration, _jerk, _snap, _orientation,
                              _angular_velocity, Eigen::Vector3d::Zero()){ }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int64_t timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
    int64_t time_from_start_ns;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d jerk;
    Eigen::Vector3d snap;

    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d angular_acceleration;

    // Accessors for making dealing with orientation/angular velocity easier.
    inline double getYaw() const { return yawFromQuaternion(orientation); }
    inline double getYawRate() const { return angular_velocity.z(); }
    inline double getYawAcc() const { return angular_acceleration.z(); }
    // WARNING: sets roll and pitch to 0.
    inline void setFromYaw(double yaw) {
        orientation = quaternionFromYaw(yaw);
    }
    inline void setFromYawRate(double yaw_rate) {
        angular_velocity.x() = 0.0;
        angular_velocity.y() = 0.0;
        angular_velocity.z() = yaw_rate;
    }
    inline void setFromYawAcc(double yaw_acc) {
        angular_acceleration.x() = 0.0;
        angular_acceleration.y() = 0.0;
        angular_acceleration.z() = yaw_acc;
    }
};

struct EigenOdometry {
    EigenOdometry()
        :timestamp_ns(-1),
         position(Eigen::Vector3d::Zero()),
         orientation(Eigen::Quaterniond::Identity()),
         velocity(Eigen::Vector3d::Zero()),
         angular_velocity(Eigen::Vector3d::Zero()) {}

    EigenOdometry(const Eigen::Vector3d& _position,
                  const Eigen::Quaterniond& _orientation,
                  const Eigen::Vector3d& _velocity_body,
                  const Eigen::Vector3d& _angular_velocity)
        :position(_position),
         orientation(_orientation),
         velocity(_velocity_body),
         angular_velocity(_angular_velocity) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int64_t timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angular_velocity;
    Eigen::Matrix<double, 6, 6> pose_covariance_;
    Eigen::Matrix<double, 6, 6> twist_covariance_;

    inline Eigen::VectorXd getVel() const
    {
        Eigen::VectorXd vel = Eigen::VectorXd::Zero(6);
        vel.block<3, 1>(0, 0) = velocity;
        vel.block<3, 1>(3, 0) = angular_velocity;
        return vel;
    }
    
    inline Eigen::VectorXd getPosAtt() const
    {
        Eigen::Vector3d rpy;
        getEulerAnglesFromQuaternion(orientation, &rpy);
        Eigen::VectorXd posatt = Eigen::VectorXd::Zero(6);
        posatt.block<3, 1>(0, 0) = position;
        posatt.block<3, 1>(3, 0) = rpy;
        return posatt;
    }
};

} //namespace pyramid_msgs

#endif //PYRAMID_MSGS_PYRAMID_EIGEN_MSGS_H
