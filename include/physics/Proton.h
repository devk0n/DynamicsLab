#ifndef PROTON_H
#define PROTON_H

#include <Eigen/Dense>

namespace Proton {

#define PI 3.14159265358979323846

using UniqueID = std::uint16_t;

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using VectorXd = Eigen::VectorXd;

using Matrix2d = Eigen::Matrix2d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Matrix5d = Eigen::Matrix<double, 5, 5>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using MatrixXd = Eigen::MatrixXd;

using MatrixG = Eigen::Matrix<double, 3, 4>;
using MatrixL = Eigen::Matrix<double, 3, 4>;

using Quaterniond = Eigen::Quaterniond;

inline void toggle(bool& flag) { flag = !flag; }

inline Matrix4d omegaMatrix(const Vector3d& w) {
  Matrix4d Omega;
  Omega <<      0, - w.x(), - w.y(), - w.z(),
            w.x(),       0,   w.z(), - w.y(),
            w.y(), - w.z(),       0,   w.x(),
            w.z(),   w.y(), - w.x(),       0;
  return Omega;
}

inline Vector4d deltaThetaToQuaternion(const Vector3d& delta_theta) {
  double theta_sq = delta_theta.squaredNorm();
  if (theta_sq < 1e-10) {
    return Vector4d(1, 0, 0, 0); // Identity quaternion
  }
  double theta = sqrt(theta_sq);
  Vector3d axis = delta_theta / theta;
  double half_theta = 0.5 * theta;
  double sinc_theta = (abs(theta) > 1e-6) ? sin(half_theta) / theta : 0.5;
  return Vector4d(cos(half_theta), sinc_theta * delta_theta.x(), sinc_theta * delta_theta.y(), sinc_theta * delta_theta.z());
}

inline Vector4d quaternionMultiply(const Vector4d& a, const Vector4d& b) {
  return Vector4d(
      a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(),
      a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
      a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x(),
      a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w()
  );
}

inline Vector4d applySmallRotationQuaternion(const Vector4d& q, const Vector3d& dTheta) {
  // q is [w, x, y, z].
  // dTheta is a small rotation in radians about some axis (in world space, typically).

  // Build the 4×4 "Omega" matrix from dTheta
  Matrix4d Omega = Matrix4d::Zero();
  // For convenience:
  double dx = dTheta.x(), dy = dTheta.y(), dz = dTheta.z();

  Omega <<  0,   -dx,   -dy,   -dz,
            dx,    0,    dz,   -dy,
            dy,   -dz,    0,    dx,
            dz,    dy,   -dx,    0;

  // For a small rotation, d(q) = 0.5 * Omega * q
  Vector4d dq = 0.5 * Omega * q;

  // Apply the correction
  Vector4d q_new = q + dq;
  q_new.normalize();
  return q_new;
}


inline Vector4d integrateQuaternion(const Vector4d& q, const Vector3d& omega, double dt) {
  Matrix4d Omega = omegaMatrix(omega);
  Vector4d dq = 0.5 * Omega * q;
  Vector4d q_new = q + dt * dq;
  q_new.normalize();
  return q_new;
}

inline Vector4d integrateQuaternionExp(const Vector4d& q, const Vector3d& omega, double dt) {
  double angle = omega.norm() * dt;
  if (angle < 1e-8) return q;
  Vector3d axis = omega.normalized();
  double half = 0.5 * angle;
  double sin_half = std::sin(half);
  Vector4d delta_q;
  delta_q << std::cos(half), sin_half * axis;
  Vector4d result;
  result << q[0] * delta_q[0] - q.segment<3>(1).dot(delta_q.segment<3>(1)),
            q[0] * delta_q.segment<3>(1) + delta_q[0] * q.segment<3>(1) + q.segment<3>(1).cross(delta_q.segment<3>(1));
  result.normalize();
  return result;
}

inline Matrix3d skew(const Vector3d& v) {
  return (Matrix3d() <<
     0,   -v.z(),  v.y(),
     v.z(),   0,  -v.x(),
    -v.y(), v.x(),   0).finished();
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
  R << 1 - 2 * (y*y + z*z),     2 * (x*y - w*z),     2 * (x*z + w*y),
           2 * (x*y + w*z), 1 - 2 * (x*x + z*z),     2 * (y*z - w*x),
           2 * (x*z - w*y),     2 * (y*z + w*x), 1 - 2 * (x*x + y*y);

  return R;
}

inline Matrix3d updateInertiaWorld(const Vector4d& orientation, const Vector3d& inverseInertia) {
  Matrix3d R = quaternionToRotationMatrix(orientation);
  return R * inverseInertia.asDiagonal() * R.transpose();
}

inline Vector4d eulerToQuaternion(const Vector3d& eulerAngles) {
  double roll  = eulerAngles.x();
  double pitch = eulerAngles.y();
  double yaw   = eulerAngles.z();

  double cy = std::cos(  yaw * 0.5);
  double sy = std::sin(  yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos( roll * 0.5);
  double sr = std::sin( roll * 0.5);

  Vector4d q;
  q[0] = cr * cp * cy + sr * sp * sy;
  q[1] = sr * cp * cy - cr * sp * sy;
  q[2] = cr * sp * cy + sr * cp * sy;
  q[3] = cr * cp * sy - sr * sp * cy;

  return q;
}

inline double cosd(double degrees) {
  double radians = degrees * PI / 180.0;
  return std::cos(radians);
}

inline double sind(double degrees) {
  double radians = degrees * PI / 180.0;
  return std::sin(radians);
}

inline Vector4d eulerToQuaternionDegrees(const Vector3d& eulerAnglesDegrees) {
  constexpr double degToRad = PI / 180.0;
  double roll  = eulerAnglesDegrees.z() * degToRad;
  double pitch = eulerAnglesDegrees.y() * degToRad;
  double yaw   = eulerAnglesDegrees.x() * degToRad;

  double cy = std::cos(yaw   * 0.5);
  double sy = std::sin(yaw   * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll  * 0.5);
  double sr = std::sin(roll  * 0.5);

  Vector4d q;
  q[0] = cr * cp * cy + sr * sp * sy;
  q[1] = sr * cp * cy - cr * sp * sy;
  q[2] = cr * sp * cy + sr * cp * sy;
  q[3] = cr * cp * sy - sr * sp * cy;

  return q;
}

} // Proton

#endif // PROTON_H
