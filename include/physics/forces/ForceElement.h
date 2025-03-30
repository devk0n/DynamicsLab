#ifndef FORCE_ELEMENT_H
#define FORCE_ELEMENT_H

#include <Eigen/Dense>

namespace Proton {

class ForceElement {
public:
  virtual ~ForceElement() = default;

  // Compute force and Jacobian for implicit integration
  virtual void computeForceAndJacobian(
      Eigen::VectorXd& F_ext,     // Output force vector
      Eigen::MatrixXd& K,         // Output stiffness matrix
      int dof_dq                  // Total number of velocity DOFs
  ) = 0;
};

} // Proton

#endif // FORCE_ELEMENT_H
