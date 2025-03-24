#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "Proton.h"

#include "Body.h"
#include "Constraint.h"
#include "ForceGenerator.h"

namespace Proton {
class Dynamics {
public:
  Dynamics() = default;

  // Body management
  UniqueID addBody(
    const double &mass,
    const Vector3d &inertia,
    const Vector3d &position,
    const Vector4d &orientation
  );

  Body* getBody(UniqueID ID);
  const Body *getBody(UniqueID ID) const;

  // Force handling
  void addForceGenerator(const std::shared_ptr<ForceGenerator>& generator) {
    m_forceGenerators.emplace_back(generator);
  }

  // Constraint handling
  void addConstraint(const std::shared_ptr<Constraint>& constraint) {
    m_constraints.emplace_back(constraint);
    m_numConstraints += constraint->getDOFs();
  }

  const std::vector<std::unique_ptr<Body>>& getBodies() const { return m_bodies; }

  std::vector<std::shared_ptr<Constraint>> getConstraints() const {
    return m_constraints;
  }

  void step(double dt);

private:

  VectorXd getPositionState() const;
  VectorXd getVelocityState() const;

  // System state
  std::vector<std::unique_ptr<Body>> m_bodies;
  std::unordered_map<UniqueID, size_t> m_bodyIndex;
  int m_numBodies = 0;

  // Force generators
  std::vector<std::shared_ptr<ForceGenerator>> m_forceGenerators;

  // constraints
  std::vector<std::shared_ptr<Constraint>> m_constraints;
  int m_numConstraints = 0;

  UniqueID m_nextID = 0;
};
} // Proton

#endif // DYNAMICS_H
