#ifndef PROTON_H
#define PROTON_H

#include <Eigen/Dense>

namespace Proton {

using UniqueID = std::uint16_t;

using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using VectorXd = Eigen::VectorXd;

using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using MatrixXd = Eigen::MatrixXd;

using MatrixG = Eigen::Matrix<double, 3, 4>;
using MatrixL = Eigen::Matrix<double, 3, 4>;

using Quaterniond = Eigen::Quaterniond;

inline void toggle(bool& flag) { flag = !flag; }

inline Matrix4d omegaMatrix(const Vector3d& w) {
  Matrix4d Omega;
  Omega <<  0,    -w.x(), -w.y(), -w.z(),
            w.x(),  0,     w.z(), -w.y(),
            w.y(), -w.z(), 0,     w.x(),
            w.z(),  w.y(), -w.x(), 0;
  return Omega;
}

inline Vector4d applySmallRotationQuaternion(const Vector4d& q, const Vector3d& deltaTheta) {
  const Matrix4d Omega = omegaMatrix(deltaTheta);
  const Vector4d dq = 0.5 * Omega * q;
  Vector4d q_new = q + dq;
  q_new.normalize();
  return q_new;
}

inline Vector4d integrateQuaternion(const Vector4d& q, const Vector3d& omega, double dt) {
  const Matrix4d Omega = omegaMatrix(omega);
  const Vector4d dq = 0.5 * Omega * q;
  Vector4d q_new = q + dt * dq;
  q_new.normalize();
  return q_new;
}

inline Matrix3d skew(const Vector3d& v) {
  Matrix3d S;
  S <<     0,   -v.z(),  v.y(),
         v.z(),     0,  -v.x(),
        -v.y(),  v.x(),     0;
  return S;
}

inline MatrixG matrixG(Vector4d e) {
  MatrixG G;
  G << -e[1],  e[0], -e[3],  e[2],
       -e[2],  e[3],  e[0], -e[1],
       -e[3], -e[2],  e[1],  e[0];
  return G;
}

inline MatrixL matrixL(Vector4d e) {
  MatrixG L;
  L << -e[1],  e[0],  e[3], -e[2],
       -e[2], -e[3],  e[0],  e[1],
       -e[3],  e[2], -e[1],  e[0];
  return L;
}

inline Matrix3d quaternionToRotationMatrix(const Vector4d& q) {
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];

  Matrix3d R;
  R << 1 - 2*(y*y + z*z),   2*(x*y - w*z),     2*(x*z + w*y),
       2*(x*y + w*z),     1 - 2*(x*x + z*z),   2*(y*z - w*x),
       2*(x*z - w*y),       2*(y*z + w*x),   1 - 2*(x*x + y*y);

  return R;
}

inline Matrix3d updateInertiaWorld(const Vector4d& orientation, const Vector3d& inverseInertia) {
  Matrix3d R = quaternionToRotationMatrix(orientation);
  return R * inverseInertia.asDiagonal() * R.transpose();
}


} // Proton

#endif // PROTON_H
