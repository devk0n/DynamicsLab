
#include "tools.h"
#include <cmath>

// Function definition
Eigen::Vector4d eulerToQuaternion(double roll, double pitch, double yaw) {
   double cy = cos(yaw * 0.5);
   double sy = sin(yaw * 0.5);
   double cp = cos(pitch * 0.5);
   double sp = sin(pitch * 0.5);
   double cr = cos(roll * 0.5);
   double sr = sin(roll * 0.5);

   // Return as Eigen::Vector4d [w, x, y, z]
   return Eigen::Vector4d(
       cr * cp * cy + sr * sp * sy, // w
       sr * cp * cy - cr * sp * sy, // x
       cr * sp * cy + sr * cp * sy, // y
       cr * cp * sy - sr * sp * cy  // z
   );
}