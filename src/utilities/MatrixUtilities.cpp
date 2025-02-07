
#include "MatrixUtilities.h"

Eigen::Matrix<double, 3, 4> MatrixUtilities::transformationMatrixL(Eigen::Vector4d transformationMatrix) {

    double w = transformationMatrix(0);
    double x = transformationMatrix(1);
    double y = transformationMatrix(2);
    double z = transformationMatrix(3);

    Eigen::Matrix<double, 3, 4> L;
    L <<  -x,  w,  z, -y,
          -y, -z,  w,  x,
          -z,  y, -x,  w;

    return L;
}

Eigen::Matrix<double, 3, 4> MatrixUtilities::transformationMatrixG(Eigen::Vector4d transformationMatrix) {

    double w = transformationMatrix(0);
    double x = transformationMatrix(1);
    double y = transformationMatrix(2);
    double z = transformationMatrix(3);

    Eigen::Matrix<double, 3, 4> G;

    G << -x,  w, -z,  y,
         -y,  z,  w, -x,
         -z, -y,  x,  w;

    return G;
}