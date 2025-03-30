#include "Dynamics.h"

namespace Proton {

void Dynamics::step(double dt) const {

  if (dt > 0.01) {
    LOG_WARN("Too large time step: ", dt, " Setting to 0.01 to keep stability.");
    dt = 0.01;
  }

  const int dof_q = m_numBodies * 7;   // 3 position + 4 quaternion (w, x, y, z)
  const int dof_dq = m_numBodies * 6;  // 3 linear + 3 angular velocity

  VectorXd q_n(dof_q), dq_n(dof_dq);
  for (int i = 0; i < m_numBodies; ++i) {
    const auto& body = m_bodies[i];
    q_n.segment<3>(i * 7) = body->getPosition();
    q_n.segment<4>(i * 7 + 3) = body->getOrientation(); // (w, x, y, z)
    dq_n.segment<3>(i * 6) = body->getLinearVelocity();
    dq_n.segment<3>(i * 6 + 3) = body->getAngularVelocity();
  }

  VectorXd q_next = q_n;
  VectorXd dq_next = dq_n;

  constexpr int maxIters = 10;
  constexpr double tol = 1e-8;

  for (int iter = 0; iter < maxIters; ++iter) {
    const VectorXd q_mid = 0.5 * (q_n + q_next);
    const VectorXd dq_mid = 0.5 * (dq_n + dq_next);

    // Update body states to midpoint
    for (int i = 0; i < m_numBodies; ++i) {
      auto& body = m_bodies[i];
      body->clearForces();
      body->clearTorque();

      if (body->isFixed()) continue;

      Vector4d q = q_mid.segment<4>(i * 7 + 3);
      q.normalize();
      body->setOrientation(q);

      body->updateInertiaWorld();
      body->setPosition(q_mid.segment<3>(i * 7));

      body->setLinearVelocity(dq_mid.segment<3>(i * 6));
      body->setAngularVelocity(dq_mid.segment<3>(i * 6 + 3));

    }

    for (const auto& fg : m_forceGenerators) {
      fg->apply(dt);
    }

    // External forces and torques
    VectorXd F_ext = VectorXd::Zero(dof_dq);
    for (int i = 0; i < m_numBodies; ++i) {
      const auto& body = m_bodies[i];
      F_ext.segment<3>(i * 6) = body->getForce();
      F_ext.segment<3>(i * 6 + 3) = body->getTorque();

      F_ext.segment<3>(i * 6 + 3) -= skew(body->getAngularVelocity()) * body->getInertia().asDiagonal() * body->getAngularVelocity();
    }

    // Mass and inertia matrix
    MatrixXd M = MatrixXd::Zero(dof_dq, dof_dq);
    for (int i = 0; i < m_numBodies; ++i) {
      const auto& body = m_bodies[i];
      M.block<3, 3>(i * 6, i * 6) = body->getMass() * Matrix3d::Identity();
      M.block<3, 3>(i * 6 + 3, i * 6 + 3) = body->getInertiaWorld();
    }

    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(dof_dq, dof_dq);

    // Apply all force elements
    for (const auto& fe : m_forceElements) {
      fe->computeForceAndJacobian(F_ext, K, dof_dq);
    }

    // Constraints
    MatrixXd P = MatrixXd::Zero(m_numConstraints, dof_dq);
    VectorXd gamma = VectorXd::Zero(m_numConstraints);
    int row = 0;
    for (const auto& c : m_constraints) {
      c->computeJacobian(P, row);
      c->computeAccelerationCorrection(gamma, row);
      row += c->getDOFs();
    }

    // Solve KKT system
    MatrixXd KKT(dof_dq + m_numConstraints, dof_dq + m_numConstraints);
    KKT.setZero();
    KKT.topLeftCorner(dof_dq, dof_dq) = M - dt * dt * K;
    KKT.topRightCorner(dof_dq, m_numConstraints) = P.transpose();
    KKT.bottomLeftCorner(m_numConstraints, dof_dq) = P;

    VectorXd rhs(dof_dq + m_numConstraints);
    rhs.head(dof_dq) = F_ext;
    rhs.tail(m_numConstraints) = gamma;

    VectorXd sol = KKT.fullPivLu().solve(rhs);
    VectorXd ddq_mid = sol.head(dof_dq);

    // Midpoint integration
    VectorXd dq_new = dq_n + dt * ddq_mid;
    VectorXd q_new = q_n;

    for (int i = 0; i < m_numBodies; ++i) {
      // Linear position update
      q_new.segment<3>(i * 7) += dt * 0.5 * (dq_n.segment<3>(i * 6) + dq_new.segment<3>(i * 6));

      Vector4d q = q_n.segment<4>(i * 7 + 3);
      Vector3d omega = 0.5 * (dq_n.segment<3>(i * 6 + 3) + dq_new.segment<3>(i * 6 + 3));

      Vector4d q_updated = integrateQuaternionExp(q, omega, dt);
      q_new.segment<4>(i * 7 + 3) = q_updated;

    }

    double err = (dq_new - dq_next).norm() + (q_new - q_next).norm();
    dq_next = dq_new;
    q_next = q_new;
    if (err < tol) break;
  }

  projectConstraints(q_next, dq_next, dof_dq, dt);
  writeBack(q_next, dq_next);
}

void Dynamics::projectConstraints(VectorXd& q_next, VectorXd& dq_next, int dof_dq, double dt) const {
  constexpr int maxProjectionIters = 3;
  constexpr double projectionTol = 1e-5;

  for (int iter = 0; iter < maxProjectionIters; ++iter) {
    // Update body states for constraint evaluation
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

    // Compute constraints
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
      MatrixXd JJt = J * J.transpose();
      Eigen::FullPivLU<MatrixXd> solver(JJt);

      // Position projection: q^c = q'' - Jᵀ(JJᵀ)⁻¹Φ
      VectorXd lambda_p = solver.solve(phi);
      VectorXd delta_q = J.transpose() * lambda_p;

      // Velocity projection: v^c = v'' - Jᵀ(JJᵀ)⁻¹Jv
      VectorXd lambda_v = solver.solve(Jdq);
      VectorXd delta_dq = J.transpose() * lambda_v;

      // Apply corrections
      for (int i = 0; i < m_numBodies; ++i) {
        if (m_bodies[i]->isFixed()) continue;

        // Position correction
        q_next.segment<3>(i * 7) -= delta_q.segment<3>(i * 6);
        Vector3d delta_theta = delta_q.segment<3>(i * 6 + 3);
        Vector4d q = q_next.segment<4>(i * 7 + 3);
        q_next.segment<4>(i * 7 + 3) = integrateQuaternionExp(q, delta_theta, dt);

        // Velocity correction
        dq_next.segment<3>(i * 6) -= delta_dq.segment<3>(i * 6);
        dq_next.segment<3>(i * 6 + 3) -= delta_dq.segment<3>(i * 6 + 3);
      }
    }
  }
}

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

VectorXd Dynamics::getPositionState() const {
  VectorXd positionState(m_numBodies * 3);

  int i = 0;
  for (const auto &body : m_bodies) {
    positionState.segment<3>(i * 3) = body->getPosition();
    ++i;
  }
  return positionState;
}

VectorXd Dynamics::getVelocityState() const {
  VectorXd velocityState(m_numBodies * 3);

  int i = 0;
  for (const auto &body : m_bodies) {
    velocityState.segment<3>(i * 3) = body->getLinearVelocity();
    ++i;
  }
  return velocityState;
}

UniqueID Dynamics::addBody() {
  UniqueID ID = m_nextID++;
  m_bodies.emplace_back(std::make_unique<Body>(ID, m_numBodies));
  m_bodyIndex.try_emplace(ID, m_bodies.size() - 1);
  m_numBodies++;
  LOG_DEBUG("Added Body with ID: ", ID);
  return ID;
}

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

const Body *Dynamics::getBody(const UniqueID ID) const {
  auto it = m_bodyIndex.find(ID);
  if (it != m_bodyIndex.end()) {
    if (it->second < m_bodies.size()) {
      return m_bodies[it->second].get();
    }
  }
  LOG_WARN("Body ID ", ID, " not found.");
  return nullptr;
}

} // Proton
