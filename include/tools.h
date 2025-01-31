//
// Created by devkon on 30/01/2025.
//

#ifndef DYNAMICSLAB_TOOLS_H
#define DYNAMICSLAB_TOOLS_H

#include <Eigen/Dense>

using namespace Eigen;

Matrix<double, 3, 4> transformationMatrixL(Vector4d transformationMatrix);

Matrix<double, 3, 4> transformationMatrixG(Vector4d transformationMatrix);

Vector4d eulerToQuaternion(double roll, double pitch, double yaw);

Vector3d quaternionToEuler(const Vector4d &quaternion);



#endif //DYNAMICSLAB_TOOLS_H
