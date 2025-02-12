#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H

#include <vector>
#include "RigidBody.h"
#include "Solver.h"

class PhysicsEngine {
public:
  PhysicsEngine() : PhysicsEngine(0.004166666) {}

  explicit PhysicsEngine(double timeStep);

  void initialize();

  void step();

  void start();

  void stop();

  Eigen::VectorXd computeStateDerivatives(const Eigen::VectorXd &state);

  // Adders
  void addRigidBody(const RigidBody &rigidBody);

  // Getters
  std::vector<RigidBody> &getRigidBodies();

  // Setters
  void setExternalForces(Eigen::Vector3d externalForces);

  [[nodiscard]] bool isInitialized() const;

  [[nodiscard]] bool isRunning() const;

private:

  Solver m_solver;

  Eigen::MatrixXd m_matrixA;
  Eigen::VectorXd m_vectorX;
  Eigen::VectorXd m_externalForces;

  double m_timeStep;

  bool m_initialized;
  bool m_running;

  std::vector<RigidBody> m_rigidBodies;
};

#endif // PHYSICSENGINE_H
