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
    m_AngularVelocity(1) = (std::rand() % 11) - 5;
    m_AngularVelocity(2) = (std::rand() % 11) - 5;
    m_AngularVelocity(3) = (std::rand() % 11) - 5;

    m_Velocity(0) = (std::rand() % 11) - 5;
    m_Velocity(1) = (std::rand() % 11) - 5;
    m_Velocity(2) = (std::rand() % 11) - 5;
}

Matrix<double, 3, 4> RigidBody::getLTransformationMatrix(Vector4d transformationMatrix) {

    double w = transformationMatrix(0);
    double x = transformationMatrix(1);
    double y = transformationMatrix(2);
    double z = transformationMatrix(3);

    Matrix<double, 3, 4> L;
    L <<  -x,  w,  z, -y,
          -y, -z,  w,  x,
          -z,  y, -x,  w;

    return L;
}

Matrix<double, 3, 4> RigidBody::getGTransformationMatrix(Vector4d transformationMatrix) {

    double w = transformationMatrix(0);
    double x = transformationMatrix(1);
    double y = transformationMatrix(2);
    double z = transformationMatrix(3);

    Matrix<double, 3, 4> G;

    G << -x,  w, -z,  y,
         -y,  z,  w, -x,
         -z, -y,  x,  w;

    return G;
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
    return m_AngularVelocity;
}

Vector3d RigidBody::getVelocity() {
    return m_Velocity;
}

void RigidBody::setPosition(Vector3d position) {
    m_Position = position;
}

void RigidBody::setAngularVelocity(Vector4d angularVelocity) {
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
    return 4 * getLTransformationMatrix(m_Orientation).transpose() * m_GlobalInertiaTensor * getLTransformationMatrix(m_Orientation);
}

// Matrix4d inertiaTensor = 4 * transformationMatrix.transpose() * globalInertiaTensor * transformationMatrix;

