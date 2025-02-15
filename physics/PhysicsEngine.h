#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H

#include <vector>
#include "RigidBody.h"
#include "GroundPoint.h"
#include <Eigen/Dense>

class PhysicsEngine {
public:
  PhysicsEngine() : PhysicsEngine(0.00001) {
  }

  explicit PhysicsEngine(double timeStep);

  void initialize();

  void step();

  void start();

  void stop();

  void reset();

  void addRigidBody(const RigidBody &rigidBody);

  void addGroundPoint(const GroundPoint &groundPoint);

  std::vector<RigidBody> &getRigidBodies();

  std::vector<GroundPoint> &getGroundPoints();

  void setGravity(const Eigen::Vector3d &gravity);

  void setConstraintMatrix(const Eigen::MatrixXd &P);

  void setConstraintViolation(const Eigen::VectorXd &c);

  [[nodiscard]] bool isInitialized() const;

  [[nodiscard]] bool isRunning() const;

private:
  Eigen::MatrixXd m_matrixA; // Mass matrix (M)
  Eigen::VectorXd m_vectorX; // State vector (positions, velocities)
  Eigen::Vector3d m_gravity; // Gravity vector (fixed-size)
  Eigen::MatrixXd m_P; // Constraint matrix (P)
  Eigen::VectorXd m_c; // Constraint violation vector (c)

  double m_timeStep;
  bool m_initialized;
  bool m_running;

  std::vector<RigidBody> m_rigidBodies;
  std::vector<GroundPoint> m_groundPoints;
  std::vector<Eigen::Vector3d> m_accelerations; // Store accelerations for each rigid body
};

#endif // PHYSICSENGINE_H
