#include "rigid_body.h"


RigidBody::RigidBody(double mass, const Eigen::Matrix3d& inertia, const Eigen::Vector3d& position, const Eigen::Vector4d& orientation)
    : m_Mass(mass),
      m_Inertia(inertia),
      m_Position(position),
      m_Orientation(orientation),
      m_Velocity(Eigen::Vector3d::Zero()),
      m_AngularVelocity(Eigen::Vector4d::Zero()) {}


Eigen::Vector3d RigidBody::getPosition() const {
    return m_Position;
}


Eigen::Vector4d RigidBody::getOrientation() const {
    return m_Orientation;
}
