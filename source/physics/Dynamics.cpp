#include "Dynamics.h"

namespace Proton {

// Main simulation step using implicit midpoint integration
void Dynamics::step(double dt) const {
  // Clamp timestep to ensure numerical stability
  if (dt > 0.01) {
    LOG_WARN("Too large time step: ", dt, " Setting to 0.01 to keep stability.");
    dt = 0.01;
  }

  // Generalized position DoFs: 3 pos + 4 orientation (quaternion) per body
  const int dof_q  = m_numBodies * 7;

  // Generalized velocity DoFs: 3 linear + 3 angular per body
  const int dof_dq = m_numBodies * 6;

  // Initial state at timestep n
  VectorXd q_n(dof_q), dq_n(dof_dq);
  initializeState(q_n, dq_n);

  // Initialize next state with current state
  VectorXd q_next = q_n;
  VectorXd dq_next = dq_n;

  constexpr int maxIters = 10;   // Max number of nonlinear solver iterations
  constexpr double tol = 1e-8;   // Convergence tolerance

  // Iteratively solve the implicit midpoint equations
  for (int iter = 0; iter < maxIters; ++iter) {
    // Compute midpoint states
    const VectorXd q_mid = 0.5 * (q_n + q_next);
    const VectorXd dq_mid = 0.5 * (dq_n + dq_next);

    // Update all body states to midpoint for evaluating forces and constraints
    updateMidpointState(q_mid, dq_mid);

    // Apply force generators (like gravity or drag)
    for (const auto& fg : m_forceGenerators) {
      fg->apply(dt);
    }

    // External force vector (forces and torques)
    VectorXd F_ext = VectorXd::Zero(dof_dq);
    computeExternalForces(F_ext);

    // Assemble mass matrix (block-diagonal)
    MatrixXd M = MatrixXd::Zero(dof_dq, dof_dq);
    assembleMassMatrix(M);

    // Apply force elements (like springs/dampers) and compute Jacobian K
    MatrixXd K = MatrixXd::Zero(dof_dq, dof_dq);
    applyForceElements(F_ext, K);

    // Assemble constraint Jacobian P and correction term gamma
    MatrixXd P = MatrixXd::Zero(m_numConstraints, dof_dq);
    VectorXd gamma = VectorXd::Zero(m_numConstraints);
    assembleConstraints(P, gamma);

    // Solve KKT system to get acceleration at midpoint
    VectorXd ddq_mid = solveKKTSystem(M, K, P, F_ext, gamma, dt);

    // Update velocity with midpoint acceleration
    VectorXd dq_new = dq_n + dt * ddq_mid;

    // Integrate position and orientation using midpoint rule
    integrateStateMidpoint(q_n, dq_n, dq_new, dt, q_next);

    // Update velocity for next iteration
    dq_next = dq_new;

    // Check for convergence
    double err = (dq_new - dq_next).norm() + (q_next - q_n).norm();
    if (err < tol) break;
  }

  // Project any constraint violations (e.g., enforce joints)
  projectConstraints(q_next, dq_next, dof_dq, dt);

  // Save new state back to the rigid bodies
  writeBack(q_next, dq_next);
}

// Store current generalized state (q, dq) from body data
void Dynamics::initializeState(VectorXd& q, VectorXd& dq) const {
  for (int i = 0; i < m_numBodies; ++i) {
    const auto& body = m_bodies[i];
    q.segment<3>(i * 7)       = body->getPosition();
    q.segment<4>(i * 7 + 3)   = body->getOrientation();
    dq.segment<3>(i * 6)      = body->getLinearVelocity();
    dq.segment<3>(i * 6 + 3)  = body->getAngularVelocity();
  }
}

// Set all bodies to the midpoint state (used for force/constraint eval)
void Dynamics::updateMidpointState(const VectorXd& q_mid, const VectorXd& dq_mid) const {
  for (int i = 0; i < m_numBodies; ++i) {
    auto& body = m_bodies[i];

    if (body->isFixed()) continue;

    body->clearForces();
    body->clearTorque();

    body->setPosition(q_mid.segment<3>(i * 7));
    body->setOrientation(q_mid.segment<4>(i * 7 + 3).normalized());
    body->setLinearVelocity(dq_mid.segment<3>(i * 6));
    body->setAngularVelocity(dq_mid.segment<3>(i * 6 + 3));
    body->updateInertiaWorld();
  }
}

// Gather external forces and torques into a vector
void Dynamics::computeExternalForces(VectorXd& F_ext) const {
  for (int i = 0; i < m_numBodies; ++i) {
    const auto& body = m_bodies[i];

    F_ext.segment<3>(i * 6)     = body->getForce();
    F_ext.segment<3>(i * 6 + 3) = body->getTorque()
                                - skew(body->getAngularVelocity())
                                * body->getInertia().asDiagonal()
                                * body->getAngularVelocity(); // Gyroscopic term
  }
}

// Fill the mass matrix with mass and inertia tensors
void Dynamics::assembleMassMatrix(Eigen::Ref<MatrixXd> M) const {
  for (int i = 0; i < m_numBodies; ++i) {
    const auto& body = m_bodies[i];
    M.block<3, 3>(i * 6, i * 6)         = body->getMass() * Matrix3d::Identity();
    M.block<3, 3>(i * 6 + 3, i * 6 + 3) = body->getInertiaWorld();
  }
}

// Apply all nonlinear force elements (springs, actuators, etc.)
void Dynamics::applyForceElements(VectorXd& F_ext, MatrixXd& K) const {
  for (const auto& fe : m_forceElements) {
    fe->computeForceAndJacobian(F_ext, K, static_cast<int>(F_ext.size()));
  }
}

// Assemble constraint Jacobian matrix P and RHS correction vector gamma
void Dynamics::assembleConstraints(MatrixXd& P, VectorXd& gamma) const {
  int row = 0;
  for (const auto& c : m_constraints) {
    c->computeJacobian(P, row);
    c->computeAccelerationCorrection(gamma, row);
    row += c->getDOFs();
  }
}

// Solve the Karush-Kuhn-Tucker system for constrained acceleration
VectorXd Dynamics::solveKKTSystem(
  const MatrixXd& M,
  const MatrixXd& K,
  const MatrixXd& P,
  const VectorXd& F_ext,
  const VectorXd& gamma,
  double dt
) {
  const int dof_dq = static_cast<int>(F_ext.size());
  const int nc = static_cast<int>(gamma.size());

  MatrixXd KKT(dof_dq + nc, dof_dq + nc);
  KKT.setZero();

  // Top-left: mass and stiffness
  KKT.topLeftCorner(dof_dq, dof_dq).noalias() = M - dt * dt * K;
  // Top-right and bottom-left: constraint Jacobian
  KKT.topRightCorner(dof_dq, nc) = P.transpose();
  KKT.bottomLeftCorner(nc, dof_dq) = P;

  // Right-hand side: external forces and constraint correction
  VectorXd rhs(dof_dq + nc);
  rhs.head(dof_dq) = F_ext;
  rhs.tail(nc) = gamma;

  // Solve the linear system (can be optimized later)
  VectorXd sol = KKT.fullPivLu().solve(rhs);
  return sol.head(dof_dq); // Only care about acceleration
}

// Midpoint integration of position and quaternion orientation
void Dynamics::integrateStateMidpoint(
  const VectorXd& q_n,
  const VectorXd& dq_n,
  const VectorXd& dq_new,
  double dt,
  VectorXd& q_next
) const {
  for (int i = 0; i < m_numBodies; ++i) {
    // Linear position: average velocity
    Vector3d v_avg = 0.5 * (dq_n.segment<3>(i * 6) + dq_new.segment<3>(i * 6));
    q_next.segment<3>(i * 7) = q_n.segment<3>(i * 7) + dt * v_avg;

    // Quaternion orientation: integrate using exponential map
    Vector4d q0 = q_n.segment<4>(i * 7 + 3);
    Vector3d omega_avg = 0.5 * (dq_n.segment<3>(i * 6 + 3) + dq_new.segment<3>(i * 6 + 3));
    q_next.segment<4>(i * 7 + 3) = integrateQuaternionExp(q0, omega_avg, dt);
  }
}

// Projects positions and velocities to satisfy constraints exactly
void Dynamics::projectConstraints(VectorXd& q_next, VectorXd& dq_next, int dof_dq, double dt) const {
  constexpr int maxProjectionIters = 3;
  constexpr double projectionTol = 1e-5;

  for (int iter = 0; iter < maxProjectionIters; ++iter) {
    // Update bodies to new state
    for (int i = 0; i < m_numBodies; ++i) {
      if (m_bodies[i]->isFixed()) continue;
      m_bodies[i]->setPosition(q_next.segment<3>(i * 7));
      Vector4d q = q_next.segment<4>(i * 7 + 3);
      q.normalize();
      m_bodies[i]->setOrientation(q);
      m_bodies[i]->setLinearVelocity(dq_next.segment<3>(i * 6));
      m_bodies[i]->setAngularVelocity(dq_next.segment<3>(i * 6 + 3));
      m_bodies[i]->updateInertiaWorld();
    }

    // Evaluate constraint position errors and velocity drift
    VectorXd phi = VectorXd::Zero(m_numConstraints);
    VectorXd Jdq = VectorXd::Zero(m_numConstraints);
    MatrixXd J   = MatrixXd::Zero(m_numConstraints, dof_dq);

    int row = 0;
    for (const auto& c : m_constraints) {
      c->computePositionError(phi, row);
      c->computeJacobian(J, row);
      Jdq.segment(row, c->getDOFs()) = J.block(row, 0, c->getDOFs(), dof_dq) * dq_next;
      row += c->getDOFs();
    }

    double totalError = phi.squaredNorm() + Jdq.squaredNorm();
    if (totalError < projectionTol) break;

    if (m_numConstraints > 0) {
      // Solve for Lagrange multipliers using normal equations
      MatrixXd JJt = J * J.transpose();
      Eigen::FullPivLU<MatrixXd> solver(JJt);

      // Project positions
      VectorXd lambda_p = solver.solve(phi);
      VectorXd delta_q = J.transpose() * lambda_p;

      // Project velocities
      VectorXd lambda_v = solver.solve(Jdq);
      VectorXd delta_dq = J.transpose() * lambda_v;

      // Apply corrections to each body
      for (int i = 0; i < m_numBodies; ++i) {
        if (m_bodies[i]->isFixed()) continue;

        q_next.segment<3>(i * 7) -= delta_q.segment<3>(i * 6);
        Vector3d delta_theta = delta_q.segment<3>(i * 6 + 3);
        Vector4d q = q_next.segment<4>(i * 7 + 3);
        q_next.segment<4>(i * 7 + 3) = integrateQuaternionExp(q, delta_theta, dt);

        dq_next.segment<3>(i * 6) -= delta_dq.segment<3>(i * 6);
        dq_next.segment<3>(i * 6 + 3) -= delta_dq.segment<3>(i * 6 + 3);
      }
    }
  }
}

// Write the final integrated state back to the body objects
void Dynamics::writeBack(VectorXd q_next, VectorXd dq_next) const {
  for (int i = 0; i < m_numBodies; ++i) {
    auto& body = m_bodies[i];
    if (body->isFixed()) continue;

    Vector4d q = q_next.segment<4>(i * 7 + 3);
    q.normalize();
    body->setOrientation(q);
    body->setAngularVelocity(dq_next.segment<3>(i * 6 + 3));
    body->setPosition(q_next.segment<3>(i * 7));
    body->setLinearVelocity(dq_next.segment<3>(i * 6));
  }
}

// Add a new dynamic body and return its ID
UniqueID Dynamics::addBody() {
  UniqueID ID = m_nextID++;
  m_bodies.emplace_back(std::make_unique<Body>(ID, m_numBodies));
  m_bodyIndex.try_emplace(ID, m_bodies.size() - 1);
  m_numBodies++;
  LOG_DEBUG("Added Body with ID: ", ID);
  return ID;
}

// Retrieve non-const pointer to a body by ID
Body* Dynamics::getBody(const UniqueID ID) {
  auto it = m_bodyIndex.find(ID);
  if (it != m_bodyIndex.end() && it->second < m_bodies.size()) {
    Body* b = m_bodies[it->second].get();
    LOG_DEBUG("Retrieved Body with ID: ", ID, " at pointer ", static_cast<void *>(b));
    return b;
  }
  LOG_WARN("Body ID ", ID, " not found.");
  return nullptr;
}

// Retrieve const pointer to a body by ID
const Body* Dynamics::getBody(const UniqueID ID) const {
  auto it = m_bodyIndex.find(ID);
  if (it != m_bodyIndex.end()) {
    if (it->second < m_bodies.size()) {
      return m_bodies[it->second].get();
    }
  }
  LOG_WARN("Body ID ", ID, " not found.");
  return nullptr;
}

} // namespace Proton
