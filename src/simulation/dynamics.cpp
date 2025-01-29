#include <iostream>
#include "dynamics.h"

Dynamics::Dynamics() {}

void Dynamics::addBody(const std::shared_ptr<RigidBody>& body) {
    m_Bodies.push_back(body);
    initializeSize();
    initializeContent();
    std::cout << "Added body. Total count: " << m_Bodies.size() << std::endl;
}

void Dynamics::initializeContent() {
    if (m_Bodies.empty()) return;
    int b = getBodyCount();

    for (int i = 0; i < b; i++) {
        // Matrix A
        m_SystemMassInertiaMatrix.block(7 * i, 7 * i, 3, 3) = m_Bodies[i]->getMassMatrix();
        m_SystemMassInertiaMatrix.block(7 * i + 4, 7 * i + 4, 3, 3) = m_Bodies[i]->getGlobalInertiaTensor();

        m_QuaternionConstraintMatrix.block(1 * i, 3 + (7 * i), 1, 4) = m_Bodies[i]->getOrientation().transpose();

        // Matrix B
        auto Ld = m_Bodies[i]->getLTransformationMatrix(m_Bodies[i]->getAngularVelocity());
        auto Jm = m_Bodies[i]->getGlobalInertiaTensor();
        auto L = m_Bodies[i]->getLTransformationMatrix(m_Bodies[i]->getOrientation());
        auto H = 4 * Ld.transpose() * Jm * Ld;

        m_VelocityDependentTerm.middleRows(3, 4) = 2 * H * m_Bodies[i]->getAngularVelocity();
        m_QuaternionNormSquared.row(i) = m_Bodies[i]->getOrientation().transpose() * m_Bodies[i]->getOrientation();

        m_GeneralizedExternalForces.setZero();
    }

    m_A.topLeftCorner(7 * b, 7 * b) = m_SystemMassInertiaMatrix;
    m_A.bottomLeftCorner(b, 7 * b) = m_QuaternionConstraintMatrix;
    m_A.topRightCorner(7 * b, b) = m_QuaternionConstraintMatrix.transpose();

}

void Dynamics::initializeSize() {
    if (m_Bodies.empty()) return;
    int b = getBodyCount();

    m_SystemMassInertiaMatrix.resize(7 * b, 7 * b);
    m_QuaternionConstraintMatrix.resize(1 * b, 7 * b);

    m_GeneralizedCoordinates.resize(7 * b, 1);
    m_GeneralizedVelocities.resize(7 * b, 1);
    m_GeneralizedAccelerations.resize(7 * b, 1);

    m_VelocityDependentTerm.resize(7 * b);
    m_QuaternionNormSquared.resize(b, 1);
    m_GeneralizedExternalForces.resize(7 * b, 1);

    m_A.resize(7 * b + b, 7 * b + b);
    m_AZeros.resize(b, b);

    m_B.resize(7 * b + b, 1);
    m_BZeros.resize(b, 1);

    m_X.resize(7 * b + b, 1);

    m_SystemMassInertiaMatrix.setZero();
    m_QuaternionConstraintMatrix.setZero();
    m_AZeros.setZero();
    m_GeneralizedCoordinates.setZero();
    m_GeneralizedVelocities.setZero();
    m_GeneralizedAccelerations.setZero();
    m_VelocityDependentTerm.setZero();
    m_QuaternionNormSquared.setZero();
    m_GeneralizedExternalForces.setZero();

    m_A.setZero();
    m_B.setZero();
    m_X.setZero();

}

void Dynamics::step(double deltaTime) {

}

int Dynamics::getBodyCount() {
    return static_cast<int>(m_Bodies.size());
}

std::shared_ptr<RigidBody>& Dynamics::getBody(int index) {
    return m_Bodies[index];
}

void Dynamics::debug() {
    std::cout << "Dynamics Initialized" << std::endl;
    std::cout << "System Mass Inertia Matrix: " << m_SystemMassInertiaMatrix.rows() << " " << m_SystemMassInertiaMatrix.cols() << std::endl;
    std::cout << m_SystemMassInertiaMatrix << std::endl;
    std::cout << "Quaternion Constraint Matrix: " << m_QuaternionConstraintMatrix.rows() << " " << m_QuaternionConstraintMatrix.cols() << std::endl;
    std::cout << m_QuaternionConstraintMatrix << std::endl;

    std::cout << "AZeros: " << m_AZeros.rows() << " " << m_AZeros.cols() << std::endl;
    std::cout << m_AZeros << std::endl;



    std::cout << "Generalized Coordinates: " << m_GeneralizedCoordinates.rows() << " " << m_GeneralizedCoordinates.cols() << std::endl;
    std::cout << m_GeneralizedCoordinates << std::endl;
    std::cout << "Generalized Velocities: " << m_GeneralizedVelocities.rows() << " " << m_GeneralizedVelocities.cols() << std::endl;
    std::cout << m_GeneralizedVelocities << std::endl;
    std::cout << "Generalized Accelerations: " << m_GeneralizedAccelerations.rows() << " " << m_GeneralizedAccelerations.cols() << std::endl;
    std::cout << m_GeneralizedAccelerations << std::endl;

    std::cout << "Velocity Dependent Term: " << m_VelocityDependentTerm.rows() << " " << m_VelocityDependentTerm.cols() << std::endl;
    std::cout << m_VelocityDependentTerm << std::endl;
    std::cout << "Quaternion Norm Squared: " << m_QuaternionNormSquared.rows() << " " << m_QuaternionNormSquared.cols() << std::endl;
    std::cout << m_QuaternionNormSquared << std::endl;
    std::cout << "Generalized External Forces: " << m_GeneralizedExternalForces.rows() << " " << m_GeneralizedExternalForces.cols() << std::endl;
    std::cout << m_GeneralizedExternalForces << std::endl;

    std::cout << "A: " << m_A.rows() << " " << m_A.cols() << std::endl;
    std::cout << m_A << std::endl;
    std::cout << "B: " << m_B.rows() << " " << m_B.cols() << std::endl;
    std::cout << m_B << std::endl;
    std::cout << "X: " << m_X.rows() << " " << m_X.cols() << std::endl;
    std::cout << m_X << std::endl;
}

Eigen::MatrixXd Dynamics::getMatrixA() {
    return m_A;
}

Eigen::MatrixXd Dynamics::getSystemMassInertiaMatrix() {
    return m_SystemMassInertiaMatrix;
}

Eigen::MatrixXd Dynamics::getQuaternionConstraintMatrix() {
    return m_QuaternionConstraintMatrix;
}

Eigen::MatrixXd Dynamics::getGeneralizedCoordinates() {
    return m_GeneralizedCoordinates;
}

Eigen::MatrixXd Dynamics::getGeneralizedVelocities() {
    return m_GeneralizedVelocities;
}

Eigen::MatrixXd Dynamics::getGeneralizedAccelerations() {
    return m_GeneralizedAccelerations;
}

Eigen::VectorXd Dynamics::getVelocityDependentTerm() {
    return m_VelocityDependentTerm;
}

Eigen::MatrixXd Dynamics::getQuaternionNormSquared() {
    return m_QuaternionNormSquared;
}

Eigen::MatrixXd Dynamics::getGeneralizedExternalForces() {
    return m_GeneralizedExternalForces;
}

Eigen::MatrixXd Dynamics::getAZeros() {
    return m_AZeros;
}

Eigen::MatrixXd Dynamics::getMatrixB() {
    return m_B;
}

Eigen::MatrixXd Dynamics::getMatrixX() {
    return m_X;
}
