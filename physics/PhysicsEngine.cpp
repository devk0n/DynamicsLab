#include "PhysicsEngine.h"

PhysicsEngine::PhysicsEngine(double timeStep)
    : m_solver(timeStep), m_timeStep(timeStep), m_initialized(false), m_running(false) {}

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
  int l = static_cast<int>(m_rigidBodies.size());
  int m = l * 7;

  // Matrix A
  m_matrixA.setZero(m + l, m + l);
  // Vector B
  m_externalForces.setZero(m + l, 1);
  // Vector X
  m_vectorX.setZero(m + l, 1);

  for (int i = 0; i < l; i++) {
    int n = 7 * i;
    m_matrixA.block(n, n, 3, 3) = m_rigidBodies[i].getMassMatrix();
  }
  m_initialized = true;
}

void PhysicsEngine::step() {
  if (m_rigidBodies.empty()) return;

  // Define the current state vector (positions, orientations, velocities, angular velocities)
  Eigen::VectorXd state(
      m_rigidBodies.size() * 14); // 7 DOF per body (position + orientation + velocity + angular velocity)
  for (size_t i = 0; i < m_rigidBodies.size(); i++) {
    int n = 14 * i;
    state.segment(n, 3) = m_rigidBodies[i].getPosition();
    state.segment(n + 3, 4) = m_rigidBodies[i].getOrientation();
    state.segment(n + 7, 3) = m_rigidBodies[i].getLinearVelocity();
    state.segment(n + 10, 4) = m_rigidBodies[i].getAngularVelocity();
  }

  // Perform integration using the Solver class
  auto newState = m_solver.integrateStep(state, [this](const Eigen::VectorXd &state) {
    return computeStateDerivatives(state);
  });

  // Update the rigid bodies with the new state
  for (size_t i = 0; i < m_rigidBodies.size(); i++) {
    int n = 14 * i;
    m_rigidBodies[i].setPosition(newState.segment(n, 3));
    m_rigidBodies[i].setOrientation(newState.segment(n + 3, 4));
    m_rigidBodies[i].setLinearVelocity(newState.segment(n + 7, 3));
    m_rigidBodies[i].setAngularVelocity(newState.segment(n + 10, 4));
  }
}

Eigen::VectorXd PhysicsEngine::computeStateDerivatives(const Eigen::VectorXd &state) {
  Eigen::VectorXd derivatives(state.size());
  derivatives.setZero();

  // Compute the derivatives for each rigid body
  for (size_t i = 0; i < m_rigidBodies.size(); i++) {
    int n = 7 * i;

    // Extract position, orientation, velocity, and angular velocity from the state vector
    [[maybe_unused]] Eigen::Vector3d position = state.segment(n, 3);
    [[maybe_unused]] Eigen::Vector4d orientation = state.segment(n + 3, 4);
    Eigen::Vector3d velocity = state.segment(n + 7, 3);
    Eigen::Vector4d angularVelocity = state.segment(n + 10, 4);

    // Compute accelerations (linear and angular)
    Eigen::Vector3d linearAcceleration = m_externalForces / m_rigidBodies[i].getMass();
    Eigen::Vector4d angularAcceleration = Eigen::Vector4d::Zero(); // Simplified for now

    // Update the derivatives
    derivatives.segment(n, 3) = velocity; // Derivative of position is velocity
    derivatives.segment(n + 3, 4) = angularVelocity; // Simplified for now
    derivatives.segment(n + 7, 3) = linearAcceleration; // Derivative of velocity is acceleration
    derivatives.segment(n + 10, 4) = angularAcceleration; // Simplified for now
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
