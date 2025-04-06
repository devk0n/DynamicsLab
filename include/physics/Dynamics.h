#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "Proton.h"

#include "Body.h"
#include "Constraint.h"
#include "ForceGenerator.h"

namespace Proton {
class Dynamics {
public:

  // Body management
  UniqueID addBody();

  Body* getBody(UniqueID ID);
  [[nodiscard]] const Body *getBody(UniqueID ID) const;
  [[nodiscard]] const std::vector<std::unique_ptr<Body>>& getBodies() const { return m_bodies; }

  // Force handling
  void addForceGenerator(const std::shared_ptr<ForceGenerator>& generator) {
    m_forceGenerators.emplace_back(generator);
  }

  [[nodiscard]] const std::vector<std::shared_ptr<ForceGenerator>>& getForceGenerators() const {
    return m_forceGenerators;
  }

  void addForceElement(const std::shared_ptr<ForceElement>& element) {
    m_forceElements.emplace_back(element);
  }

  [[nodiscard]] const std::vector<std::shared_ptr<ForceElement>>& getForceElements() const {
    return m_forceElements;
  }

  // Constraint handling
  void addConstraint(const std::shared_ptr<Constraint>& constraint) {
    m_constraints.emplace_back(constraint);
    m_numConstraints += constraint->getDOFs();
  }

  [[nodiscard]] std::vector<std::shared_ptr<Constraint>> getConstraints() const {
    return m_constraints;
  }

  double clampTimeStep(double dt) const;

  void step(double dt) const;

private:

  const double m_maxTimeStep = 0.01;

  const int m_maxIters = 5;   // Max number of nonlinear solver iterations
  const double m_tol = 1e-10;   // Convergence tolerance

  const int m_maxProjectionIters = 3;
  const double m_projectionTol = 1e-10;

  void initializeState(VectorXd& q, VectorXd& dq) const;
  void updateMidpointState(const VectorXd& q_mid, const VectorXd& dq_mid) const;
  void computeExternalForces(VectorXd& F_ext) const;
  void assembleMassMatrix(Eigen::Ref<MatrixXd> M) const;
  void applyForceElements(VectorXd &F_ext, MatrixXd &K) const;
  void assembleConstraints(MatrixXd& P, VectorXd& gamma) const;
  [[nodiscard]] static VectorXd solveKKTSystem(
    const MatrixXd& M, const MatrixXd& K,
    const MatrixXd& P, const VectorXd& F_ext,
    const VectorXd& gamma, double dt
  );
  void integrateStateMidpoint(
    const VectorXd& q_n, const VectorXd& dq_n,
    const VectorXd& dq_new, double dt,
    VectorXd& q_next
  ) const;
  void writeBack(VectorXd q_next, VectorXd dq_next) const;
  void projectConstraints(VectorXd &q_next, VectorXd &dq_next, int dof_dq, double dt) const;

  // System state
  std::vector<std::unique_ptr<Body>> m_bodies{};
  std::unordered_map<UniqueID, size_t> m_bodyIndex{};
  int m_numBodies = 0;

  // Force generators
  std::vector<std::shared_ptr<ForceGenerator>> m_forceGenerators{};

  // Force elements
  std::vector<std::shared_ptr<ForceElement>> m_forceElements{};

  // Constraints
  std::vector<std::shared_ptr<Constraint>> m_constraints;
  int m_numConstraints = 0;

  UniqueID m_nextID = 0;
};
} // Proton
#endif // DYNAMICS_H
