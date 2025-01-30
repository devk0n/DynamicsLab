//
// Created by devkon on 30/01/2025.
//

#ifndef DYNAMICSLAB_TOOLS_H
#define DYNAMICSLAB_TOOLS_H

#include <Eigen/Dense>

Eigen::Vector4d eulerToQuaternion(double roll, double pitch, double yaw);

Eigen::Vector3d quaternionToEuler(const Eigen::Vector4d &quaternion);

#endif //DYNAMICSLAB_TOOLS_H
