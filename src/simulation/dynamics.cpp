#include <iostream>
#include <utility>

#include "dynamics.h"
#include "tools.h"

/*
    SystemMassInertiaMatrix         (M*)
    QuaternionConstraintMatrix      (P)

    GeneralizedCoordinates          (q)
    GeneralizedVelocities           (qd)
    GeneralizedAccelerations        (qdd)

    VelocityDependentTerm           (b*)

    QuaternionNormSquared           (c)
    GeneralizedExternalForces       (g*)
 */
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
    int m;

    for (int i = 0; i < b; i++) {
        m = 7 * i;
        m_GeneralizedCoordinates.segment(m, 3) = m_Bodies[i]->getPosition();
        m_GeneralizedCoordinates.segment(m + 3, 4) = m_Bodies[i]->getOrientation();
        m_GeneralizedVelocities.segment(m, 3) = m_Bodies[i]->getVelocity();
        m_GeneralizedVelocities.segment(m + 3, 4) = m_Bodies[i]->getAngularVelocity();
        m_QuaternionNormSquared(i) = m_Bodies[i]->getQuaternionNormSquared();

        // Matrix A
        m_SystemMassInertiaMatrix.block(m, m, 3, 3) = m_Bodies[i]->getMassMatrix();
        m_SystemMassInertiaMatrix.block(m + 3, m + 3, 4, 4) = m_Bodies[i]->getInertiaTensor();

        m_QuaternionConstraintMatrix.block(i, 3 + m, 1, 4) = m_Bodies[i]->getOrientation().transpose();

        // Matrix B
        auto Ld = transformationMatrixL(m_Bodies[i]->getAngularVelocity());
        auto Jm = m_Bodies[i]->getGlobalInertiaTensor();
        auto L = transformationMatrixL(m_Bodies[i]->getOrientation());
        auto H = 4 * Ld.transpose() * Jm * L;

        m_VelocityDependentTerm.middleRows(3, 4) = 2 * H * m_Bodies[i]->getAngularVelocity();

        m_GeneralizedExternalForces.tail<4>() = 2 * transformationMatrixG(m_Bodies[i]->getOrientation()).transpose() * m_ExternalTorques;

        m_B.segment(0, 7 * b) = m_GeneralizedExternalForces - m_VelocityDependentTerm;
        m_B.tail(b) = -m_QuaternionNormSquared;

        m_Bodies[i]->normalizeOrientation();
    }

    m_A.topLeftCorner(7 * b, 7 * b) = m_SystemMassInertiaMatrix;
    m_A.bottomLeftCorner(b, 7 * b) = m_QuaternionConstraintMatrix;
    m_A.topRightCorner(7 * b, b) = m_QuaternionConstraintMatrix.transpose();
}

void Dynamics::step(double deltaTime) {
    if (!m_isSimulationRunning) return;
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
    int m = 7 * b;

    m_SystemMassInertiaMatrix.resize(m, m);
    m_QuaternionConstraintMatrix.resize(b, m);

    m_GeneralizedCoordinates.resize(m);
    m_GeneralizedVelocities.resize(m);
    m_GeneralizedAccelerations.resize(m);

    m_VelocityDependentTerm.resize(m);
    m_QuaternionNormSquared.resize(b);
    m_GeneralizedExternalForces.resize(m);

    m_A.resize(m + b, m + b);
    m_B.resize(m + b);
    m_X.resize(m + b);

    m_SystemMassInertiaMatrix.setZero();
    m_QuaternionConstraintMatrix.setZero();

    m_GeneralizedCoordinates.setZero();
    m_GeneralizedVelocities.setZero();
    m_GeneralizedAccelerations.setZero();

    m_VelocityDependentTerm.setZero();
    m_QuaternionNormSquared.setZero();
    m_GeneralizedExternalForces.setZero();

    m_ExternalForces.setZero();
    m_ExternalTorques.setZero();

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

void Dynamics::setExternalTorques(Vector3d externalTorques) {
    m_ExternalTorques = externalTorques;
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

void Dynamics::startSimulation() {
    m_isSimulationRunning = true;
}

void Dynamics::stopSimulation() {
    m_isSimulationRunning = false;
}

void Dynamics::resetSimulation() {
    m_isSimulationRunning = true;
    for (auto &body : m_Bodies) {
        body->setPosition(body->getInitialPosition());
        body->setOrientation(body->getInitialOrientation());
        body->setVelocity(body->getInitialVelocity());
        body->setAngularVelocity(body->getInitialAngularVelocity());
    }
    initializeSize();
    initializeContent();
}
