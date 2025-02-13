#include "PhysicsEngine.h"

PhysicsEngine::PhysicsEngine(double timeStep)
  : m_solver(timeStep), m_timeStep(timeStep), m_initialized(false), m_running(false) {
}

void PhysicsEngine::addRigidBody(const RigidBody &rigidBody) {
  m_rigidBodies.push_back(rigidBody);
}

std::vector<RigidBody> &PhysicsEngine::getRigidBodies() {
  return m_rigidBodies;
}

void PhysicsEngine::setExternalForces(Eigen::Vector3d externalForces) {
  m_externalForces = externalForces;
}

void PhysicsEngine::initialize() {
  if (m_rigidBodies.empty()) return;
  const int l = static_cast<int>(m_rigidBodies.size());
  // int m = l * 7;

  m_matrixA.resize(l * 3, l * 3);
  m_externalForces.resize(l * 3);
  m_vectorX.resize(l * 3);

  for (int i = 0; i < l; i++) {
    const int n = 3 * i;
    m_matrixA.block(n, n, 3, 3) = m_rigidBodies[i].getMassMatrix();
  }
  m_initialized = true;
}

void PhysicsEngine::step() {
  if (m_rigidBodies.empty()) return;

  constexpr int dof = 3;

  // Define the current state vector (positions, orientations, velocities, angular velocities)
  // 3 DOF for simple | 7 DOF per body (position + orientation + velocity + angular velocity)
  Eigen::VectorXd state(m_rigidBodies.size() * dof * 2);

  for (size_t i = 0; i < m_rigidBodies.size(); i++) {
    const int n = dof * i;
    state.segment(n, 3) = m_rigidBodies[i].getPosition();
    state.segment(n + dof, 3) = m_rigidBodies[i].getLinearVelocity();
  }

  // Perform integration using the Solver class
  auto newState = m_solver.integrateStep(state, [this](const Eigen::VectorXd &state) {
    return computeStateDerivatives(state);
  });

  // Update the rigid bodies with the new state
  for (size_t i = 0; i < m_rigidBodies.size(); i++) {
    int n = dof * i;
    m_rigidBodies[i].setPosition(newState.segment(n, 3));
    m_rigidBodies[i].setLinearVelocity(newState.segment(n + dof, 3));
  }
}

Eigen::VectorXd PhysicsEngine::computeStateDerivatives(const Eigen::VectorXd &state) const {
  Eigen::VectorXd derivatives(state.size());
  derivatives.setZero();

  constexpr int dof = 3;

  // Compute the derivatives for each rigid body
  for (size_t i = 0; i < m_rigidBodies.size(); i++) {
    int n = dof * i;

    // Extract position, orientation, velocity, and angular velocity from the state vector
    [[maybe_unused]] Eigen::Vector3d position = state.segment(n, 3);
    Eigen::Vector3d velocity = state.segment(n + dof, 3);

    // Compute accelerations (linear and angular)
    Eigen::Vector3d linearAcceleration = m_externalForces;

    // Update the derivatives
    derivatives.segment(n, 3) = velocity; // Derivative of position is velocity
    derivatives.segment(n + dof, 3) = linearAcceleration; // Derivative of velocity is acceleration
  }

  return derivatives;
}

void PhysicsEngine::start() {
  m_running = true;
}

void PhysicsEngine::stop() {
  m_running = false;
}

bool PhysicsEngine::isInitialized() const {
  return m_initialized;
}

bool PhysicsEngine::isRunning() const {
  return m_running;
}
