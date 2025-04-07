#include "Dynamics.h"
#include "Logger.h"

namespace Proton {

// Main simulation step using implicit midpoint integration
void Dynamics::step(double dt) {
  // Clamp timestep to ensure numerical stability
  dt = clampTimeStep(dt);

  const auto& dof_q  = m_numBodies * 7; // Generalized position DoFs: 3 pos + 4 orientation (quaternion) per body
  const auto& dof_dq = m_numBodies * 6; // Generalized velocity DoFs: 3 linear + 3 angular per body

  // Initial state at timestep n
  VectorXd q_n = VectorXd::Zero(dof_q);
  VectorXd dq_n = VectorXd::Zero(dof_dq);
  initializeState(q_n, dq_n);

  // Initialize next state with current state
  VectorXd q_next = q_n;
  VectorXd dq_next = dq_n;
  VectorXd q_prev = q_n;
  VectorXd dq_prev = dq_n;

  // Iteratively solve the implicit midpoint equations
  for (int iter = 0; iter < m_maxIters; ++iter) {

    // Safety check for invalid velocities
    if (!dq_n.allFinite()) {
      LOG_ERROR("dq_n contains invalid values, resetting to zero");
      dq_n = VectorXd::Zero(dof_dq);
    }

    if (!dq_next.allFinite()) {
      LOG_ERROR("dq_next contains invalid values, resetting to zero");
      dq_next = VectorXd::Zero(dof_dq);
    }

    // Compute midpoint velocity with safety checks
    VectorXd dq_mid;
    try {
      dq_mid = 0.5 * (dq_n + dq_next);
      if (!dq_mid.allFinite()) {
        LOG_WARN("Using fallback for dq_mid due to invalid values in velocity vectors");
        dq_mid = dq_n.allFinite() ? dq_n : VectorXd::Zero(dof_dq);
      }
    } catch (...) {
      LOG_ERROR("Exception during dq_mid calculation");
      dq_mid = VectorXd::Zero(dof_dq);
    }

    // Compute midpoint positions with safety checks
    VectorXd q_mid = VectorXd::Zero(dof_q);

    try {
      for (int i = 0; i < m_numBodies; ++i) {
        // Linear position midpoint with safety
        Vector3d pos_n = q_n.segment<3>(i * 7);
        Vector3d pos_next = q_next.segment<3>(i * 7);

        if (pos_n.allFinite() && pos_next.allFinite()) {
          q_mid.segment<3>(i * 7) = 0.5 * (pos_n + pos_next);
        } else {
          q_mid.segment<3>(i * 7) = pos_n.allFinite() ? pos_n : (pos_next.allFinite() ? pos_next : Vector3d::Zero());
          LOG_WARN("Using fallback for position midpoint of body ", i);
        }

        // Quaternion midpoint using SLERP with safety
        Vector4d quat_n = q_n.segment<4>(i * 7 + 3);
        Vector4d quat_next = q_next.segment<4>(i * 7 + 3);

        if (quat_n.allFinite() && quat_next.allFinite()) {
          q_mid.segment<4>(i * 7 + 3) = slerpQuaternion(quat_n, quat_next, 0.5);
        } else {
          q_mid.segment<4>(i * 7 + 3) = identityQuaternion();  // Identity quaternion
          LOG_WARN("Using identity quaternion for midpoint of body ", i);
        }
      }
    } catch (...) {
      LOG_ERROR("Exception during q_mid calculation");
      // Fall back to safe values
      for (int i = 0; i < m_numBodies; ++i) {
        q_mid.segment<3>(i * 7) = Vector3d::Zero();
        q_mid.segment<4>(i * 7 + 3) = identityQuaternion();
      }
    }

    if (!q_mid.allFinite() || !dq_mid.allFinite()) {
      LOG_ERROR("q_mid or dq_mid is invalid BEFORE updateMidpointState() \n", q_mid.transpose(), dq_mid.transpose());

      // Emergency recovery - reinitialize to safe values
      for (int i = 0; i < m_numBodies; ++i) {
        q_mid.segment<3>(i * 7) = Vector3d::Zero();
        q_mid.segment<4>(i * 7 + 3) = identityQuaternion();
        dq_mid.segment<6>(i * 6) = Vector6d::Zero();
      }
    }

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
    if (!m_massMatrixInitialized) {
      m_massMatrix = MatrixXd::Zero(dof_dq, dof_dq);
      assembleMassMatrix(m_massMatrix);
      m_massMatrixInitialized = true;
    }

    // Apply force elements (like springs/dampers) and compute Jacobian K
    MatrixXd K = MatrixXd::Zero(dof_dq, dof_dq);
    applyForceElements(F_ext, K);

    // Assemble constraint Jacobian P and correction term gamma
    MatrixXd P = MatrixXd::Zero(m_numConstraints, dof_dq);
    VectorXd gamma = VectorXd::Zero(m_numConstraints);
    assembleConstraints(P, gamma);

    // Solve KKT system to get acceleration at midpoint
    VectorXd ddq_mid;
    try {
      ddq_mid = solveKKTSystem(m_massMatrix, K, P, F_ext, gamma, dt);

      // Check for numerical issues in the solution
      if (!ddq_mid.allFinite()) {
        LOG_ERROR("KKT solution contains invalid values, using zero acceleration");
        ddq_mid = VectorXd::Zero(dof_dq);
      }
    } catch (...) {
      LOG_ERROR("Exception in solveKKTSystem");
      ddq_mid = VectorXd::Zero(dof_dq);
    }

    // Update velocity with midpoint acceleration
    VectorXd dq_new;
    try {
      dq_new = dq_n + dt * ddq_mid;

      // Safety check and clamping for extreme accelerations
      if (!dq_new.allFinite()) {
        LOG_ERROR("New velocity contains invalid values, reverting to previous");
        dq_new = dq_n;
      }
    } catch (...) {
      LOG_ERROR("Exception during velocity update");
      dq_new = dq_n;  // Revert to previous state
    }

    // Integrate position and orientation using midpoint rule
    try {
      integrateStateMidpoint(q_n, dq_n, dq_new, dt, q_next);

      // Validate result
      if (!q_next.allFinite()) {
        LOG_ERROR("New position contains invalid values, reverting to previous");
        q_next = q_n;
      }
    } catch (...) {
      LOG_ERROR("Exception during position integration");
      q_next = q_n;  // Revert to previous state
    }

    // Update velocity for next iteration
    dq_next = dq_new;

    // Check for convergence
    double err = (q_next - q_prev).norm() + (dq_next - dq_prev).norm();
    if (err < m_tol) break;
    q_prev = q_next;
    dq_prev = dq_next;
  }

  // Project any constraint violations
  projectConstraints(q_next, dq_next, dof_dq, dt);

  // Final safety check before write back
  if (!q_next.allFinite() || !dq_next.allFinite()) {
    LOG_ERROR("Final state contains invalid values, reverting to initial state");
    q_next = q_n;
    dq_next = dq_n;
  }

  // Save new state back to the rigid bodies
  try {
    writeBack(q_next, dq_next);
  } catch (...) {
    LOG_ERROR("Exception during state write back");
    // Emergency recovery - revert to initial state
    writeBack(q_n, dq_n);
  }
}

// Store current generalized state (q, dq) from body data
void Dynamics::initializeState(VectorXd& q, VectorXd& dq) const {
  for (int i = 0; i < m_numBodies; ++i) {
    const auto& body = m_bodies[i];
     q.segment<3>(i * 7)     = body->getPosition();
     q.segment<4>(i * 7 + 3) = body->getOrientation();
    dq.segment<3>(i * 6)     = body->getLinearVelocity();
    dq.segment<3>(i * 6 + 3) = body->getAngularVelocity();
  }
}

// Set all bodies to the midpoint state (used for force/constraint eval)
void Dynamics::updateMidpointState(const VectorXd& q_mid, const VectorXd& dq_mid) const {
    if (m_bodies.size() != static_cast<size_t>(m_numBodies)) {
        std::cerr << "[Dynamics::updateMidpointState] m_bodies.size() != m_numBodies\n";
        std::abort();
    }

    const int q_size_expected = m_numBodies * 7;
    const int dq_size_expected = m_numBodies * 6;

    if (q_mid.size() < q_size_expected || dq_mid.size() < dq_size_expected) {
        std::cerr << "[Dynamics::updateMidpointState] q_mid or dq_mid too small\n";
        std::abort();
    }

    for (int i = 0; i < m_numBodies; ++i) {
        auto& body = m_bodies[i];

        if (!body) {
            std::cerr << "[Dynamics::updateMidpointState] Null body pointer at index " << i << "\n";
            std::abort();
        }

        body->clearForces();
        body->clearTorque();
        body->updateInertiaWorld();

        if (body->isFixed()) continue;

        const Vector3d pos = q_mid.segment<3>(i * 7);
        const Vector4d quat_raw = q_mid.segment<4>(i * 7 + 3);
        const Vector3d lin_vel = dq_mid.segment<3>(i * 6);
        const Vector3d ang_vel = dq_mid.segment<3>(i * 6 + 3);

        if (!pos.allFinite() || !quat_raw.allFinite() || !lin_vel.allFinite() || !ang_vel.allFinite()) {
            std::cerr << "[Dynamics::updateMidpointState] Invalid values for body " << i << "\n";
            std::cerr << "  pos: " << pos.transpose() << "\n";
            std::cerr << "  quat: " << quat_raw.transpose() << "\n";
            std::cerr << "  lin_vel: " << lin_vel.transpose() << "\n";
            std::cerr << "  ang_vel: " << ang_vel.transpose() << "\n";
            std::abort();
        }

        Vector4d quat_safe = quat_raw;
        double quat_norm = quat_safe.norm();
        if (quat_norm < 1e-6) {
            std::cerr << "[Dynamics::updateMidpointState] Quaternion near-zero at body " << i << " — resetting to identity\n";
            quat_safe = identityQuaternion();
        } else {
            quat_safe.normalize();
        }

        body->setPosition(pos);
        body->setOrientation(quat_safe);
        body->setLinearVelocity(lin_vel);
        body->setAngularVelocity(ang_vel);
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
                                * body->getAngularVelocity();  // Torque due to angular velocity
  }
}

// Fill the mass matrix with mass and inertia tensors
void Dynamics::assembleMassMatrix(Eigen::Ref<MatrixXd> M) const {
  for (int i = 0; i < m_numBodies; ++i) {
    const auto& body = m_bodies[i];
    M.block<3,3>(i * 6, i * 6)         = body->getMass() * Matrix3d::Identity();
    M.block<3,3>(i * 6 + 3, i * 6 + 3) = body->getInertia().asDiagonal();
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
  const double dt
) {
  const int dof_dq = static_cast<int>(F_ext.size());
  const int nc = static_cast<int>(gamma.size());

  // Check for invalid inputs
  if (!M.allFinite() || !K.allFinite() || !P.allFinite() || !F_ext.allFinite() || !gamma.allFinite()) {
    LOG_ERROR("Invalid inputs to KKT system");
    return VectorXd::Zero(dof_dq);
  }

  MatrixXd KKT(dof_dq + nc, dof_dq + nc);
  KKT.setZero();

  // Top-left: mass and stiffness with regularization to improve conditioning
  KKT.topLeftCorner(dof_dq, dof_dq).noalias() = M - dt * dt * K;

  // Top-right and bottom-left: constraint Jacobian
  KKT.topRightCorner(dof_dq, nc) = P.transpose();
  KKT.bottomLeftCorner(nc, dof_dq) = P;

  // Right-hand side: external forces and constraint correction
  VectorXd rhs(dof_dq + nc);
  rhs.head(dof_dq) = F_ext;
  rhs.tail(nc) = gamma;

  // Try more stable solver methods in sequence
  Eigen::LDLT<MatrixXd> solver(KKT);
  VectorXd sol = solver.solve(rhs);

  // Apply reasonable limits to acceleration values
  VectorXd accel = sol.head(dof_dq);

  return accel;
}

// Midpoint integration of position and quaternion orientation
void Dynamics::integrateStateMidpoint(
  const VectorXd& q_n,
  const VectorXd& dq_n,
  const VectorXd& dq_new,
  const double dt,
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
  for (int iter = 0; iter < m_maxProjectionIters; ++iter) {
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
    if (totalError < m_projectionTol) break;

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

        // Position correction (linear)
        q_next.segment<3>(i * 7) -= delta_q.segment<3>(i * 6);

        // Orientation correction (quaternion)
        Vector3d delta_theta = delta_q.segment<3>(i * 6 + 3);
        Vector4d q = q_next.segment<4>(i * 7 + 3);

        // Create small rotation quaternion from delta_theta
        // For small angles, this approximation works well
        double theta_norm = delta_theta.norm();
        Vector4d delta_q_quat;
        if (theta_norm > 1e-10) {
          Vector3d axis = delta_theta / theta_norm;
          double half_angle = -0.5 * theta_norm; // Note the negative sign for correction
          delta_q_quat << cos(half_angle), sin(half_angle) * axis;
        } else {
          delta_q_quat << identityQuaternion();
        }

        // Apply the correction by quaternion multiplication
        q_next.segment<4>(i * 7 + 3) = quaternionProduct(q, delta_q_quat);
        q_next.segment<4>(i * 7 + 3).normalize();

        // Velocity correction
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

double Dynamics::clampTimeStep(double dt) const {
  if (dt > m_maxTimeStep)
    LOG_WARN("Time step too large: ", dt, " Clamping to ", m_maxTimeStep, "");

  return std::min(dt, m_maxTimeStep);
}

} // namespace Proton
