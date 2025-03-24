#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "Proton.h"

namespace Proton {

class Constraint {
public:
  explicit Constraint(const int DOFs) : m_DOFs(DOFs) {}
  virtual ~Constraint() = default;

  virtual void computePositionError(VectorXd& phi, int startRow) const = 0;
  virtual void computeJacobian(MatrixXd& jacobian, int startRow) const = 0;
  virtual void computeAccelerationCorrection(VectorXd& gamma, int startRow) const = 0;

  int getDOFs() const { return m_DOFs; }

protected:
  int m_DOFs;

};

} // Proton

#endif // CONSTRAINT_H
