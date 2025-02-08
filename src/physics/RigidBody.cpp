#include "RigidBody.h"

#include <utility>

#include "utilities/MatrixUtilities.h"

RigidBody::RigidBody(Vector3d position,
                     Vector4d orientation,
                     Matrix3d massMatrix,
                     Matrix3d localInertiaTensor) :
                     m_position(std::move(position)),
                     m_orientation(std::move(orientation)),
                     m_massMatrix(std::move(massMatrix)),
                     m_localInertiaTensor(std::move(localInertiaTensor)) {

    m_inertiaTensor = 4 * transformationMatrixL(m_orientation).transpose() * m_localInertiaTensor * transformationMatrixL(m_orientation);
    
}

// Getters
Vector3d RigidBody::getPosition() {
    return m_position;
}

Vector3d RigidBody::getVelocity() {
    return m_velocity;
}

Vector3d RigidBody::getAcceleration() {
    return m_acceleration;
}

Vector4d RigidBody::getOrientation() {
    return m_orientation;
}

Vector4d RigidBody::getAngularVelocity() {
    return m_angularVelocity;
}

Vector4d RigidBody::getAngularAcceleration() {
    return m_angularAcceleration;
}

Matrix3d RigidBody::getMassMatrix() {
    return m_massMatrix;
}

Matrix4d RigidBody::getInertiaTensor() {
    return m_inertiaTensor;
}

Matrix3d RigidBody::getLocalInertiaTensor() {
    return m_localInertiaTensor;
}

// Setters
void RigidBody::setPosition(Vector3d position) {
    m_position = position;
}

void RigidBody::setVelocity(Vector3d velocity) {
    m_velocity = velocity;
}

void RigidBody::setAcceleration(Vector3d acceleration) {
    m_acceleration = acceleration;
}

void RigidBody::setOrientation(Vector4d orientation) {
    m_orientation = orientation;
}

void RigidBody::setAngularVelocity(Vector4d angularVelocity) {
    m_angularVelocity = angularVelocity;
}

void RigidBody::setAngularAcceleration(Vector4d angularAcceleration) {
    m_angularAcceleration = angularAcceleration;
}

void RigidBody::setMassMatrix(Matrix3d massMatrix) {
    m_massMatrix = massMatrix;
}

void RigidBody::setInertiaTensor(Matrix4d inertiaTensor) {
    m_inertiaTensor = inertiaTensor;
}

double RigidBody::getMass() {
    return m_massMatrix(0, 0);
}
