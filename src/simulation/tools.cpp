#include "tools.h"
#include <cmath>

using namespace Eigen;

// Function definition
Vector4d eulerToQuaternion(double roll, double pitch, double yaw) {
   double cy = cos(yaw * 0.5);
   double sy = sin(yaw * 0.5);
   double cp = cos(pitch * 0.5);
   double sp = sin(pitch * 0.5);
   double cr = cos(roll * 0.5);
   double sr = sin(roll * 0.5);

   // Return as Vector4d [w, x, y, z]
   return Vector4d(
       cr * cp * cy + sr * sp * sy, // w
       sr * cp * cy - cr * sp * sy, // x
       cr * sp * cy + sr * cp * sy, // y
       cr * cp * sy - sr * sp * cy  // z
   );
}

// Converts a quaternion (Vector4d) into Euler angles (roll, pitch, yaw).
Vector3d quaternionToEuler(const Vector4d &quaternion) {
    // Extract quaternion elements
    double w = quaternion[0];
    double x = quaternion[1];
    double y = quaternion[2];
    double z = quaternion[3];

    // Compute roll, pitch, and yaw according to Tait-Bryan angles (ZYX convention)
    double roll  = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    double pitch = asin(2 * (w * y - z * x));
    double yaw   = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

    // Return as Vector3d
    return Vector3d(roll, pitch, yaw);
}