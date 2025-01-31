#include <cmath>

#include "tools.h"

using namespace Eigen;

Matrix<double, 3, 4> transformationMatrixL(Vector4d transformationMatrix) {

    double w = transformationMatrix(0);
    double x = transformationMatrix(1);
    double y = transformationMatrix(2);
    double z = transformationMatrix(3);

    Matrix<double, 3, 4> L;
    L <<  -x,  w,  z, -y,
          -y, -z,  w,  x,
          -z,  y, -x,  w;

    return L;
}

Matrix<double, 3, 4> transformationMatrixG(Vector4d transformationMatrix) {

    double w = transformationMatrix(0);
    double x = transformationMatrix(1);
    double y = transformationMatrix(2);
    double z = transformationMatrix(3);

    Matrix<double, 3, 4> G;

    G << -x,  w, -z,  y,
         -y,  z,  w, -x,
         -z, -y,  x,  w;

    return G;
}

Vector4d eulerToQuaternion(double roll, double pitch, double yaw) {
   double cy = cos(yaw * 0.5);
   double sy = sin(yaw * 0.5);
   double cp = cos(pitch * 0.5);
   double sp = sin(pitch * 0.5);
   double cr = cos(roll * 0.5);
   double sr = sin(roll * 0.5);

   // Return as Vector4d [w, x, y, z]
   return {
           cr * cp * cy + sr * sp * sy, // w
           sr * cp * cy - cr * sp * sy, // x
           cr * sp * cy + sr * cp * sy, // y
           cr * cp * sy - sr * sp * cy  // z
   };
}

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


    return {roll, pitch, yaw};
}