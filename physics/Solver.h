#ifndef SOLVER_H
#define SOLVER_H

#include <Eigen/Dense>

class Solver {
public:
  // Constructor: Initializes the solver with a fixed time step size
  Solver(double timeStep);

  // Performs a single integration step using the RK4 method
  template<typename DerivativeFunction>
  Eigen::VectorXd integrateStep(const Eigen::VectorXd &currentState, DerivativeFunction computeStateDerivatives);

private:
  double m_timeStep; // The fixed time step size for the solver
};

#endif // SOLVER_H