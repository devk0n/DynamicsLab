#ifndef PROTON_H
#define PROTON_H

#include <Eigen/Dense>

namespace Proton {

// Constants
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

// Type Aliases
using UniqueID = std::uint16_t;

// Eigen Types
using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using Vector5d = Eigen::Vector<double, 5>;
using Vector6d = Eigen::Vector<double, 6>;
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

inline Vector4d quaternionProduct(const Vector4d& q1, const Vector4d& q2) {
  Vector4d result;
  // q1 = [w1, x1, y1, z1], q2 = [w2, x2, y2, z2]
  result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]; // w
  result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]; // x
  result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]; // y
  result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]; // z
  return result;
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
  MatrixL L;
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

inline Vector4d identityQuaternion() { return Vector4d(1, 0, 0, 0); }

inline Vector4d slerpQuaternion(const Vector4d& q1, const Vector4d& q2, double t) {
  // Check for invalid inputs
  if (!q1.allFinite() || !q2.allFinite() || !std::isfinite(t)) {
    // Return identity quaternion as a fallback
    return identityQuaternion();
  }

  // Handle zero-length quaternions
  if (q1.norm() < 1e-8 || q2.norm() < 1e-8) {
    return identityQuaternion();
  }

  // Normalize input quaternions
  Vector4d q1n = q1.normalized();
  Vector4d q2n = q2.normalized();

  // Calculate dot product
  double dot = q1n.dot(q2n);

  // If the dot product is negative, negate one quaternion to take the shorter path
  if (dot < 0.0) {
    q2n = -q2n;
    dot = -dot;
  }

  // Clamp dot product to valid range
  dot = std::min(std::max(dot, -1.0), 1.0);

  // If quaternions are very close, use linear interpolation
  if (dot > 0.9995) {
    Vector4d result = q1n + t * (q2n - q1n);
    return result.normalized();
  }

  // Calculate angle between quaternions
  double angle = std::acos(dot);
  double sin_angle = std::sin(angle);

  // If the divisor is too close to zero, use linear interpolation
  if (std::abs(sin_angle) < 1e-8) {
    Vector4d result = q1n + t * (q2n - q1n);
    return result.normalized();
  }

  // Spherical linear interpolation formula
  double s1 = std::sin((1.0 - t) * angle) / sin_angle;
  double s2 = std::sin(t * angle) / sin_angle;

  Vector4d result = (s1 * q1n + s2 * q2n);

  // Check for invalid results
  if (!result.allFinite() || result.norm() < 1e-8) {
    return Vector4d(1.0, 0.0, 0.0, 0.0);
  }

  return result.normalized();
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

inline Vector4d eulerToQuaternionDegrees(const Vector3d &rpyDegrees) {
  constexpr double degToRad = PI / 180.0;

  // Extract roll (X), pitch (Y), and yaw (Z) from the input.
  // Tait-Bryan angles in degrees: rpyDegrees = [roll, pitch, yaw].
  double roll  = rpyDegrees.x() * degToRad;  // rotation about X
  double pitch = rpyDegrees.y() * degToRad;  // rotation about Y
  double yaw   = rpyDegrees.z() * degToRad;  // rotation about Z

  // Compute half-angle sines and cosines
  double cr = std::cos(roll  * 0.5);
  double sr = std::sin(roll  * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cy = std::cos(yaw   * 0.5);
  double sy = std::sin(yaw   * 0.5);

  // Construct quaternion in [w, x, y, z] layout
  Vector4d q;
  q[0] = cr * cp * cy + sr * sp * sy;  // w
  q[1] = sr * cp * cy - cr * sp * sy;  // x
  q[2] = cr * sp * cy + sr * cp * sy;  // y
  q[3] = cr * cp * sy - sr * sp * cy;  // z

  return q;
}

} // Proton

#endif // PROTON_H
