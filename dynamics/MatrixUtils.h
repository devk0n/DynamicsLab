#pragma once
#include <Eigen/Dense>

namespace MatrixUtils {
    Eigen::Matrix<double, 3, 4> gMatrix(const Eigen::Vector4d &p);
    Eigen::Matrix<double, 3, 4> lMatrix(const Eigen::Vector4d &p);
    Eigen::Matrix4d skewN(const Eigen::Vector3d &a);
}