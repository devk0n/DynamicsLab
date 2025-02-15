#include "PhysicsEngine.h"

PhysicsEngine::PhysicsEngine(double timeStep)
    : m_gravity(0.0, 0.0, -9.81), m_timeStep(timeStep), m_initialized(false), m_running(false) {
}

void PhysicsEngine::addRigidBody(const RigidBody &rigidBody) {
    m_rigidBodies.push_back(rigidBody);
}

std::vector<RigidBody> &PhysicsEngine::getRigidBodies() {
    return m_rigidBodies;
}

void PhysicsEngine::setGravity(const Eigen::Vector3d &gravity) {
    m_gravity = gravity;
}

void PhysicsEngine::setConstraintMatrix(const Eigen::MatrixXd &P) {
    m_P = P;
}

void PhysicsEngine::setConstraintViolation(const Eigen::VectorXd &c) {
    m_c = c;
}

void PhysicsEngine::initialize() {
    if (m_rigidBodies.empty()) return;
    const int l = static_cast<int>(m_rigidBodies.size());

    // Initialize mass matrix (M)
    m_matrixA.resize(l * 3, l * 3);
    for (int i = 0; i < l; i++) {
        const int n = 3 * i;
        m_matrixA.block(n, n, 3, 3) = m_rigidBodies[i].getMassMatrix();
    }

    // Initialize state vector (positions and velocities)
    m_vectorX.resize(l * 6); // 3 DOF for position, 3 DOF for velocity
    for (size_t i = 0; i < m_rigidBodies.size(); i++) {
        const int n = 6 * i;
        m_vectorX.segment(n, 3) = m_rigidBodies[i].getPosition();
        m_vectorX.segment(n + 3, 3) = m_rigidBodies[i].getLinearVelocity();
    }

    // Initialize accelerations
    m_accelerations.resize(m_rigidBodies.size(), Eigen::Vector3d::Zero());

    m_initialized = true;
}

void PhysicsEngine::step() {
    if (m_rigidBodies.empty()) return;

    constexpr int dof = 3;
    const double dt = 1.0 / 240.0; // Time step for 240 Hz

    // Update positions and velocities using Velocity Verlet
    for (size_t i = 0; i < m_rigidBodies.size(); i++) {
        // Get the current state of the rigid body
        Eigen::Vector3d position = m_rigidBodies[i].getPosition();
        Eigen::Vector3d velocity = m_rigidBodies[i].getLinearVelocity();
        Eigen::Vector3d acceleration = m_accelerations[i];

        // Update position: x(t + dt) = x(t) + v(t) * dt + 0.5 * a(t) * dt^2
        position += velocity * dt + 0.5 * acceleration * dt * dt;

        // Compute new acceleration: a(t + dt) = gravity
        Eigen::Vector3d newAcceleration = m_gravity;

        // Update velocity: v(t + dt) = v(t) + 0.5 * (a(t) + a(t + dt)) * dt
        velocity += 0.5 * (acceleration + newAcceleration) * dt;

        // Store the new acceleration for the next step
        m_accelerations[i] = newAcceleration;

        // Update the rigid body state
        m_rigidBodies[i].setPosition(position);
        m_rigidBodies[i].setLinearVelocity(velocity);
    }

    // Assemble external forces (F_ext, including gravity)
    Eigen::VectorXd F_ext(m_rigidBodies.size() * dof);
    for (size_t i = 0; i < m_rigidBodies.size(); i++) {
        F_ext.segment(i * dof, dof) = m_rigidBodies[i].getMass() * m_gravity;
    }

    // Handle constraints
    if (m_P.rows() == 0) {
        // No constraints, solve unconstrained system
        Eigen::VectorXd q_ddot = m_matrixA.fullPivLu().solve(F_ext);

        // Update velocities and positions
        for (size_t i = 0; i < m_rigidBodies.size(); i++) {
            int n = dof * i;
            Eigen::Vector3d acceleration = q_ddot.segment(n, 3);
            m_rigidBodies[i].setLinearVelocity(m_rigidBodies[i].getLinearVelocity() + acceleration * dt);
        }
    } else {
        // Solve constrained system
        Eigen::MatrixXd A(m_matrixA.rows() + m_P.rows(), m_matrixA.cols() + m_P.cols());
        A << m_matrixA, m_P.transpose(),
                m_P, Eigen::MatrixXd::Zero(m_P.rows(), m_P.cols());

        Eigen::VectorXd rhs(F_ext.size() + m_c.size());
        rhs << F_ext,
                m_c;

        Eigen::VectorXd x = A.fullPivLu().solve(rhs);
        Eigen::VectorXd q_ddot = x.head(m_matrixA.rows());
        Eigen::VectorXd sigma = x.tail(m_P.rows());

        // Apply constraint forces to correct velocities
        for (size_t i = 0; i < m_rigidBodies.size(); i++) {
            int n = dof * i;
            Eigen::Vector3d correction = sigma.segment(n, 3);
            m_rigidBodies[i].setLinearVelocity(m_rigidBodies[i].getLinearVelocity() + correction);
        }
    }
}

void PhysicsEngine::start() {
    m_running = true;
}

void PhysicsEngine::stop() {
    m_running = false;
}

void PhysicsEngine::reset() {
    for (auto &m_rigidBody: m_rigidBodies) {
        m_rigidBody.reset();
    }
}

bool PhysicsEngine::isInitialized() const {
    return m_initialized;
}

bool PhysicsEngine::isRunning() const {
    return m_running;
}
