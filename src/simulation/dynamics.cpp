#include <iostream>
#include "dynamics.h"

Dynamics::Dynamics() {}

void Dynamics::addBody(const std::shared_ptr<RigidBody>& body) {
    m_Bodies.push_back(body);
    initialize();
    std::cout << "Added body. Total count: " << m_Bodies.size() << std::endl;
}

void Dynamics::initialize() {
    if (m_Bodies.empty()) return;
    size_t b = m_Bodies.size();

    m_SystemMassInertiaMatrix.resize(7 * b, 7 * b);
    m_QuaternionConstraintMatrix.resize(1 * b, 7 * b);

    m_GeneralizedCoordinates.resize(7 * b, 1);
    m_GeneralizedVelocities.resize(7 * b, 1);
    m_GeneralizedAccelerations.resize(7 * b, 1);

    m_VelocityDependentTerm.resize(7 * b, 1);
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

size_t Dynamics::getBodyCount() const {
    return m_Bodies.size();
}

const std::shared_ptr<RigidBody>& Dynamics::getBody(size_t index) const {
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
