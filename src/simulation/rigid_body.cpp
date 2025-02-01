#include <utility>
#include <iostream>
#include <random>

#include "rigid_body.h"
#include "tools.h"

using namespace Eigen;

RigidBody::RigidBody(double mass, Matrix3d globalInertiaTensor, Vector3d position, Vector4d orientation) {

    m_mass = mass;
    m_massMatrix = Matrix3d::Identity() * m_mass;

    m_position = position;
    m_orientation = orientation;

    m_globalInertiaTensor = globalInertiaTensor;

    m_inertiaTensor = 4 * transformationMatrixL(m_orientation).transpose() * m_globalInertiaTensor * transformationMatrixL(m_orientation);

    m_velocity = Vector3d::Zero();
    m_angularVelocity = Vector4d::Zero();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-5.0, 5.0);

    // Assign random values to m_velocity
    m_velocity[0] = dist(gen);
    m_velocity[1] = dist(gen);
    m_velocity[2] = dist(gen);

    m_angularVelocity[0] = dist(gen);
    m_angularVelocity[1] = dist(gen);
    m_angularVelocity[2] = dist(gen);
    m_angularVelocity[3] = dist(gen);
    normalizeOrientation();

    m_initialPosition = position;
    m_initialOrientation = orientation;
    m_initialAngularVelocity = m_angularVelocity;
    m_initialVelocity = m_velocity;


}

Vector3d RigidBody::getPosition() {
    return m_position;
}

Vector4d RigidBody::getOrientation() {
    return m_orientation;
}

Matrix3d RigidBody::getMassMatrix() {
    return m_massMatrix;
}

Matrix3d RigidBody::getGlobalInertiaTensor() {
    return m_globalInertiaTensor;
}

Vector4d RigidBody::getAngularVelocity() {
    return m_angularVelocity;
}

Vector3d RigidBody::getVelocity() {
    return m_velocity;
}

void RigidBody::setPosition(Vector3d position) {
    m_position = position;
}

void RigidBody::setAngularVelocity(Vector4d angularVelocity) {
    m_angularVelocity = angularVelocity;
}

void RigidBody::setVelocity(Vector3d velocity) {
    m_velocity = velocity;
}

void RigidBody::setOrientation(Vector4d orientation) {
    m_orientation = orientation;
}

void RigidBody::setInertiaTensor(Matrix4d inertiaTensor) {
    m_inertiaTensor = inertiaTensor;
}

void RigidBody::normalizeOrientation() {
    m_orientation = m_orientation / m_orientation.norm();
}

double RigidBody::getQuaternionNormSquared() {
    return m_angularVelocity.transpose() * m_angularVelocity;
}

Matrix4d RigidBody::getInertiaTensor() {
    return m_inertiaTensor;
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
