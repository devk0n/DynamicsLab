#include "Dynamics.h"
#include "graphics/Renderer.h"

Dynamics::Dynamics(Renderer* renderer, double stepSize) : m_renderer(renderer), m_stepSize(stepSize), m_running(false) {

}

void Dynamics::step() {
    if (m_rigidBodies.empty() || !m_running) return;

    // Solve for constraint forces and accelerations
    m_vectorX = m_matrixA.fullPivLu().solve(m_vectorB);

    // Update generalized accelerations
    m_generalizedAccelerations.noalias() += m_vectorX.block(0, 0, m_bodyCount * 7, 1) * m_stepSize;

    // Update velocities and positions
    m_generalizedVelocities.noalias() += m_generalizedAccelerations * m_stepSize;
    m_generalizedCoordinates.noalias() += m_generalizedVelocities * m_stepSize;

    // **Write back to RigidBody instances**
    for (int i = 0; i < m_bodyCount; i++) {
        int n = 7 * i;

        // Update Position
        m_rigidBodies[i]->setPosition(m_generalizedCoordinates.block(n, 0, 3, 1));

        // Update Orientation (Quaternion)
        Eigen::Vector4d newOrientation = m_generalizedCoordinates.block(n + 3, 0, 4, 1);
        newOrientation.normalize();  // Ensure quaternion remains normalized
        m_rigidBodies[i]->setOrientation(newOrientation);

        // Update Linear Velocity
        m_rigidBodies[i]->setVelocity(m_generalizedVelocities.block(n, 0, 3, 1));

        // Update Angular Velocity
        m_rigidBodies[i]->setAngularVelocity(m_generalizedVelocities.block(n + 3, 0, 4, 1));
    }
}

void Dynamics::initialize() {
if (m_rigidBodies.empty()) return;
    m_bodyCount = static_cast<int>(m_rigidBodies.size());
    int l = m_bodyCount;
    int m = 7 * l;

    m_generalizedAccelerations.setZero(m);
    m_generalizedVelocities.setZero(m);
    m_generalizedCoordinates.setZero(m);

    // M*
    m_systemMassInertiaMatrix.setZero(m, m);
    // P
    m_quaternionConstraintMatrix.setZero(l, m);
    // b*
    m_velocityDependentTerm.setZero(m);
    // c
    m_quaternionNormSquared.setZero(l);
    // g*
    m_generalizedExternalForces.setZero(m);

    // Matrix A
    m_matrixA.setZero(m + l, m + l);
    // Vector B
    m_vectorB.setZero(m + l, 1);
    // Vector X
    m_vectorX.setZero(m + l, 1);

    for (int i = 0; i < m_bodyCount; i++) {
        int n = 7 * i;

        m_generalizedCoordinates.block(n, 0, 3, 1) = m_rigidBodies[i]->getPosition();

        // M*
        m_systemMassInertiaMatrix.block(n, n, 3, 3) = m_rigidBodies[i]->getMassMatrix();
        m_systemMassInertiaMatrix.block(n + 3, n + 3, 4, 4) = m_rigidBodies[i]->getInertiaTensor();

        // P
        m_quaternionConstraintMatrix.block(i, n + 3, 1, 4) = m_rigidBodies[i]->getOrientation().transpose();

        // b*
        auto Ld = transformationMatrixL(m_rigidBodies[i]->getAngularVelocity());
        auto Jm = m_rigidBodies[i]->getLocalInertiaTensor();
        auto L = transformationMatrixL(m_rigidBodies[i]->getOrientation());
        auto H = 4 * Ld.transpose() * Jm * L;

        m_velocityDependentTerm.block(n + 3, 0, 4, 1) = 2 * H * m_rigidBodies[i]->getAngularVelocity();

        // c
        m_quaternionNormSquared.block(i, 0, 1, 1) = m_rigidBodies[i]->getAngularVelocity().transpose() * m_rigidBodies[i]->getAngularVelocity();

        // g*
        // m_generalizedExternalForces.block(n, 0, 7, 1) = VectorXd::Zero(7);

        // A
        m_matrixA.block(n, n, 7, 7) = m_systemMassInertiaMatrix.block(n, n, 7, 7);

        m_matrixA.block(m + i, n, 1, 7) = m_quaternionConstraintMatrix.block(i, n, 1, 7);
        m_matrixA.block(n, m + i, 7, 1) = m_quaternionConstraintMatrix.block(i, n, 1, 7).transpose();

        // B
        m_vectorB.block(n, 0, 7, 1) = m_generalizedExternalForces.block(n, 0, 7, 1) - m_velocityDependentTerm.block(n, 0, 7, 1);
        m_vectorB.block(m + i, 0, 1, 1) = -m_quaternionNormSquared.block(i, 0, 1, 1);

    }
}

void Dynamics::addRigidBody(std::unique_ptr<RigidBody> rigidBody) {
    m_rigidBodies.push_back(std::move(rigidBody));
}

Eigen::Matrix<double, 3, 4> Dynamics::transformationMatrixL(Eigen::Vector4d transformationMatrix) {

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

VectorXd Dynamics::getGeneralizedCoordinates() {
    return m_generalizedCoordinates;
}

VectorXd Dynamics::getGeneralizedAccelerations() {
    return m_generalizedAccelerations;
}

VectorXd Dynamics::getGeneralizedVelocities() {
    return m_generalizedVelocities;
}

VectorXd Dynamics::getGeneralizedExternalForces() {
    return m_generalizedExternalForces;
}

MatrixXd Dynamics::getMatrixA() {
    return m_matrixA;
}

VectorXd Dynamics::getVectorB() {
    return m_vectorB;
}

VectorXd Dynamics::getVectorX() {
    return m_vectorX;
}

void Dynamics::setExternalForce(Matrix<double, 3, 1> matrix) {
    m_generalizedExternalForces = matrix;
}



