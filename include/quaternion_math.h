#ifndef DYNAMICSLAB_QUATERNION_MATH_H
#define DYNAMICSLAB_QUATERNION_MATH_H

#include <Eigen/Dense>

class QuaternionMath {
    static Eigen::Matrix4d skewNegativeMatrix(Eigen::Vector3d a);
    static Eigen::Matrix<double, 3, 4> gMatrix(Eigen::Vector4d p);
    static Eigen::Matrix<double, 3, 4> lMatrix(Eigen::Vector4d p);
};


#endif //DYNAMICSLAB_QUATERNION_MATH_H
