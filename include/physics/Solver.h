#ifndef SOLVER_H
#define SOLVER_H

#include <Eigen/Dense>

class Solver {
public:
  // Constructor: Initializes the solver with a fixed time step size
  explicit Solver(double timeStep);

  template<typename DerivativeFunction>
  Eigen::VectorXd integrateStep(const Eigen::VectorXd &currentState, DerivativeFunction computeStateDerivatives) {
    // Compute the four RK4 intermediate steps
    const Eigen::VectorXd k1 = computeStateDerivatives(currentState);
    const Eigen::VectorXd k2 = computeStateDerivatives(currentState + m_timeStep * k1 * 0.5);
    const Eigen::VectorXd k3 = computeStateDerivatives(currentState + m_timeStep * k2 * 0.5);
    const Eigen::VectorXd k4 = computeStateDerivatives(currentState + m_timeStep * k3);

    // Combine the intermediate steps to compute the new state
    return currentState + (m_timeStep / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }

private:
  double m_timeStep; // The fixed time step size for the solver
};

#endif // SOLVER_H
