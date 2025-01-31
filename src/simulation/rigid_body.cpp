#include <utility>

#include "rigid_body.h"
#include "tools.h"

using namespace Eigen;

RigidBody::RigidBody(Vector3d position, Vector4d orientation, Matrix3d massMatrix, Matrix3d globalInertiaTensor)
    : m_MassMatrix(std::move(massMatrix)),
      m_GlobalInertiaTensor(std::move(globalInertiaTensor)),
      m_Position(std::move(position)),
      m_Orientation(std::move(orientation)){

    m_Velocity = Vector3d::Zero();
    m_AngularVelocity = Vector4d::Zero();

    m_initialPosition = position;
    m_initialOrientation = orientation;
    m_initialAngularVelocity = m_AngularVelocity;
    m_initialVelocity = m_Velocity;
}

Vector3d RigidBody::getPosition() {
    return m_Position;
}

Vector4d RigidBody::getOrientation() {
    return m_Orientation;
}

Matrix3d RigidBody::getMassMatrix() {
    return m_MassMatrix;
}

Matrix3d RigidBody::getGlobalInertiaTensor() {
    return m_GlobalInertiaTensor;
}

Vector4d RigidBody::getAngularVelocity() {
    Vector4d angularVelocity = m_AngularVelocity;
    // angularVelocity[0] = 0.0; // Ensure the first component is 0 when retrieved
    return angularVelocity;
}

Vector3d RigidBody::getVelocity() {
    return m_Velocity;
}

void RigidBody::setPosition(Vector3d position) {
    m_Position = position;
}

void RigidBody::setAngularVelocity(Vector4d angularVelocity) {
    // angularVelocity[0] = 0.0;
    m_AngularVelocity = angularVelocity;
}

void RigidBody::setVelocity(Vector3d velocity) {
    m_Velocity = velocity;
}

void RigidBody::setOrientation(Vector4d orientation) {
    m_Orientation = orientation;
}

void RigidBody::normalizeOrientation() {
    m_Orientation / m_Orientation.norm();
}

double RigidBody::getQuaternionNormSquared() {
    return m_AngularVelocity.transpose() * m_AngularVelocity;
}

Matrix4d RigidBody::getInertiaTensor() {
    return 4 * transformationMatrixL(m_Orientation).transpose() * m_GlobalInertiaTensor * transformationMatrixL(m_Orientation);
}

Vector3d RigidBody::getInitialPosition() {
    return m_initialPosition;
}

Vector4d RigidBody::getInitialOrientation() {
    return m_initialOrientation;
}

Vector4d RigidBody::getInitialAngularVelocity() {
    return m_initialAngularVelocity;
}

Vector3d RigidBody::getInitialVelocity() {
    return m_initialVelocity;
}

// Matrix4d inertiaTensor = 4 * transformationMatrix.transpose() * globalInertiaTensor * transformationMatrix;

