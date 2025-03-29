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

  // Constraint handling
  void addConstraint(const std::shared_ptr<Constraint>& constraint) {
    m_constraints.emplace_back(constraint);
    m_numConstraints += constraint->getDOFs();
  }

  [[nodiscard]] std::vector<std::shared_ptr<Constraint>> getConstraints() const {
    return m_constraints;
  }

  void step(double dt) const;

private:

  [[nodiscard]] VectorXd getPositionState() const;
  [[nodiscard]] VectorXd getVelocityState() const;

  void writeBack(VectorXd q_next, VectorXd dq_next) const;
  void projectConstraints(VectorXd &q_next, VectorXd &dq_next, int dof_dq, double dt) const;

  // System state
  std::vector<std::unique_ptr<Body>> m_bodies;
  std::unordered_map<UniqueID, size_t> m_bodyIndex;
  int m_numBodies = 0;

  // Force generators
  std::vector<std::shared_ptr<ForceGenerator>> m_forceGenerators;

  // Constraints
  std::vector<std::shared_ptr<Constraint>> m_constraints;
  int m_numConstraints = 0;

  UniqueID m_nextID = 0;
};
} // Proton
#endif // DYNAMICS_H
