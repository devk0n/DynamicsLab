#include "Solver.h"

// Constructor: Initializes the solver with a fixed time step size
Solver::Solver(double timeStep) : m_timeStep(timeStep) {}

// Performs a single integration step using the RK4 method
template<typename DerivativeFunction>
Eigen::VectorXd Solver::integrateStep(const Eigen::VectorXd &currentState, DerivativeFunction computeStateDerivatives) {
  // Compute the four RK4 intermediate steps
  Eigen::VectorXd k1 = computeStateDerivatives(currentState);
  Eigen::VectorXd k2 = computeStateDerivatives(currentState + 0.5 * m_timeStep * k1);
  Eigen::VectorXd k3 = computeStateDerivatives(currentState + 0.5 * m_timeStep * k2);
  Eigen::VectorXd k4 = computeStateDerivatives(currentState + m_timeStep * k3);

  // Combine the intermediate steps to compute the new state
  return currentState + (m_timeStep / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}