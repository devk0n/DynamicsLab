#include <iostream>
#include "dynamics.h"

using namespace Eigen;

Dynamics::Dynamics() = default;

void Dynamics::addBody(const std::shared_ptr<RigidBody>& body) {
    m_Bodies.push_back(body);
    initializeSize();
    initializeContent();
}

void Dynamics::initializeContent() {
    if (m_Bodies.empty()) return;
    int b = getBodyCount();

    for (int i = 0; i < b; i++) {

        m_GeneralizedCoordinates.segment(7 * i, 3) = m_Bodies[i]->getPosition();
        m_GeneralizedCoordinates.segment(7 * i + 3, 4) = m_Bodies[i]->getOrientation();
        m_GeneralizedVelocities.segment(7 * i, 3) = m_Bodies[i]->getVelocity();
        m_GeneralizedVelocities.segment(7 * i + 3, 4) = m_Bodies[i]->getAngularVelocity();

        // Matrix A
        m_SystemMassInertiaMatrix.block(7 * i, 7 * i, 3, 3) = m_Bodies[i]->getMassMatrix();
        m_SystemMassInertiaMatrix.block(7 * i + 4, 7 * i + 4, 3, 3) = m_Bodies[i]->getGlobalInertiaTensor();

        m_QuaternionConstraintMatrix.block(1 * i, 3 + (7 * i), 1, 4) = m_Bodies[i]->getOrientation().transpose();

        // Matrix B
        auto Ld = m_Bodies[i]->getLTransformationMatrix(m_Bodies[i]->getAngularVelocity());
        auto Jm = m_Bodies[i]->getGlobalInertiaTensor();
        auto L = m_Bodies[i]->getLTransformationMatrix(m_Bodies[i]->getOrientation());
        auto H = 4 * Ld.transpose() * Jm * L * m_Bodies[i]->getAngularVelocity();

        m_VelocityDependentTerm.middleRows(3, 4) = 2 * H;

        m_GeneralizedExternalForces.segment(7 * i, 7) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        m_B.segment(0, 7 * b) = m_GeneralizedExternalForces - m_VelocityDependentTerm;
        m_B.tail(b) = m_QuaternionNormSquared;
    }

    m_A.topLeftCorner(7 * b, 7 * b) = m_SystemMassInertiaMatrix;
    m_A.bottomLeftCorner(b, 7 * b) = m_QuaternionConstraintMatrix;
    m_A.topRightCorner(7 * b, b) = m_QuaternionConstraintMatrix.transpose();

}

void Dynamics::step(double deltaTime) {
    int b = getBodyCount();

    initializeContent();

    m_X = m_A.partialPivLu().solve(m_B);
    m_GeneralizedAccelerations = m_X.head(7 * b);

    // Integration
    m_GeneralizedVelocities += m_GeneralizedAccelerations * deltaTime;
    m_GeneralizedCoordinates += m_GeneralizedVelocities * deltaTime;

    for (int i = 0; i < b; i++) {

        m_Bodies[i]->setPosition(m_GeneralizedCoordinates.segment(7 * i, 3));
        m_Bodies[i]->setOrientation(m_GeneralizedCoordinates.segment(7 * i + 3, 4));
        m_Bodies[i]->setVelocity(m_GeneralizedVelocities.segment(7 * i, 3));
        m_Bodies[i]->setAngularVelocity(m_GeneralizedVelocities.segment(7 * i + 3, 4));
        m_Bodies[i]->normalizeOrientation();
    }

}

void Dynamics::initializeSize() {
    if (m_Bodies.empty()) return;
    int b = getBodyCount();

    m_SystemMassInertiaMatrix.resize(7 * b, 7 * b);
    m_QuaternionConstraintMatrix.resize(1 * b, 7 * b);

    m_GeneralizedCoordinates.resize(7 * b);
    m_GeneralizedVelocities.resize(7 * b);
    m_GeneralizedAccelerations.resize(7 * b);

    m_VelocityDependentTerm.resize(7 * b);
    m_QuaternionNormSquared.resize(b);
    m_GeneralizedExternalForces.resize(7 * b);

    m_A.resize(7 * b + b, 7 * b + b);
    m_B.resize(7 * b + b, 1);
    m_X.resize(7 * b + b, 1);

    m_SystemMassInertiaMatrix.setZero();
    m_QuaternionConstraintMatrix.setZero();
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

MatrixXd Dynamics::getMatrixA() {
    return m_A;
}

MatrixXd Dynamics::getSystemMassInertiaMatrix() {
    return m_SystemMassInertiaMatrix;
}

MatrixXd Dynamics::getQuaternionConstraintMatrix() {
    return m_QuaternionConstraintMatrix;
}

VectorXd Dynamics::getGeneralizedCoordinates() {
    return m_GeneralizedCoordinates;
}

VectorXd Dynamics::getGeneralizedVelocities() {
    return m_GeneralizedVelocities;
}

VectorXd Dynamics::getGeneralizedAccelerations() {
    return m_GeneralizedAccelerations;
}

VectorXd Dynamics::getVelocityDependentTerm() {
    return m_VelocityDependentTerm;
}

VectorXd Dynamics::getQuaternionNormSquared() {
    return m_QuaternionNormSquared;
}

VectorXd Dynamics::getGeneralizedExternalForces() {
    return m_GeneralizedExternalForces;
}

VectorXd Dynamics::getMatrixB() {
    return m_B;
}

VectorXd Dynamics::getMatrixX() {
    return m_X;
}
