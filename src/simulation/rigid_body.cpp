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

Eigen::Matrix<double, 3, 4> RigidBody::m_LTransformationMatrix() {

    double w = m_Orientation(0);
    double x = m_Orientation(1);
    double y = m_Orientation(2);
    double z = m_Orientation(3);

    Eigen::Matrix<double, 3, 4> L;
    L <<  -x,  w,  z, -y,
          -y, -z,  w,  x,
          -z,  y, -x,  w;

    return L;
}

Eigen::Matrix<double, 3, 4> RigidBody::m_GTransformationMatrix() {

    double w = m_Orientation(0);
    double x = m_Orientation(1);
    double y = m_Orientation(2);
    double z = m_Orientation(3);

    Eigen::Matrix<double, 3, 4> G;

    G << -x,  w, -z,  y,
         -y,  z,  w, -x,
         -z, -y,  x,  w;

    return G;
}

// Eigen::Matrix4d inertiaTensor = 4 * transformationMatrix.transpose() * globalInertiaTensor * transformationMatrix;

