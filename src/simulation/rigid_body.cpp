#include <iostream>
#include <utility>
#include "rigid_body.h"

RigidBody::RigidBody(Eigen::Vector3d position, Eigen::Vector4d orientation, Eigen::Matrix3d massMatrix, Eigen::Matrix3d globalInertiaTensor)
    : m_MassMatrix(std::move(massMatrix)),
      m_GlobalInertiaTensor(std::move(globalInertiaTensor)),
      m_Position(std::move(position)),
      m_Orientation(std::move(orientation)){

    m_Velocity = Eigen::Vector3d::Zero();
    m_AngularVelocity = Eigen::Vector4d::Zero();

    std::cout << "RigidBody constructor" << std::endl;
    std::cout << "Position:" << std::endl;
    std::cout << m_Position << std::endl;
    std::cout << "Orientation:" << std::endl;
    std::cout << m_Orientation << std::endl;
    std::cout << "Mass Matrix:" << std::endl;
    std::cout << m_MassMatrix << std::endl;
    std::cout << "Global Inertia Tensor:" << std::endl;
    std::cout << m_GlobalInertiaTensor << std::endl;
}

Eigen::Matrix<double, 3, 4> RigidBody::getLTransformationMatrix(Eigen::Vector4d transformationMatrix) {

    double w = transformationMatrix(0);
    double x = transformationMatrix(1);
    double y = transformationMatrix(2);
    double z = transformationMatrix(3);

    Eigen::Matrix<double, 3, 4> L;
    L <<  -x,  w,  z, -y,
          -y, -z,  w,  x,
          -z,  y, -x,  w;

    return L;
}

Eigen::Matrix<double, 3, 4> RigidBody::getGTransformationMatrix(Eigen::Vector4d transformationMatrix) {

    double w = transformationMatrix(0);
    double x = transformationMatrix(1);
    double y = transformationMatrix(2);
    double z = transformationMatrix(3);

    Eigen::Matrix<double, 3, 4> G;

    G << -x,  w, -z,  y,
         -y,  z,  w, -x,
         -z, -y,  x,  w;

    return G;
}

Eigen::Vector3d RigidBody::getPosition() {
    return m_Position;
}

Eigen::Vector4d RigidBody::getOrientation() {
    return m_Orientation;
}

Eigen::Matrix3d RigidBody::getMassMatrix() {
    return m_MassMatrix;
}

Eigen::Matrix3d RigidBody::getGlobalInertiaTensor() {
    return m_GlobalInertiaTensor;
}

Eigen::Vector4d RigidBody::getAngularVelocity() {
    return m_AngularVelocity;
}

Eigen::Vector3d RigidBody::getVelocity() {
    return m_Velocity;
}

// Eigen::Matrix4d inertiaTensor = 4 * transformationMatrix.transpose() * globalInertiaTensor * transformationMatrix;

