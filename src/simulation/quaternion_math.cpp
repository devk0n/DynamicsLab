#include "quaternion_math.h"

Eigen::Matrix4d skewMatrix(Eigen::Vector3d a) {
    Eigen::Matrix4d Sn = Eigen::Matrix4d::Zero();

    Sn.block<1, 3>(0, 1) = -a.transpose();
    Sn.block<3, 1>(1, 0) = a;
    Sn.block<3, 3>(1, 1) = -Eigen::Matrix3d((Eigen::Matrix3d() << 0, -a(2), a(1),
                                                    a(2), 0, -a(0),
                                                   -a(1), a(0), 0).finished());

    return Sn;
}

Eigen::Matrix<double, 3, 4> gMatrix(Eigen::Vector4d p) {
    Eigen::Matrix<double, 3, 4> G;

    G << -p(1), p(0), -p(3), p(2),
         -p(2), p(3), p(0), -p(1),
         -p(3), -p(2), p(1), p(0);

    return G;
}

Eigen::Matrix<double, 3, 4> lMatrix(Eigen::Vector4d p) {
    Eigen::Matrix<double, 3, 4> L;

    L << -p(1), p(0), p(3), -p(2),
         -p(2), -p(3), p(0), p(1),
         -p(3), p(2), -p(1), p(0);

    return L;
}
