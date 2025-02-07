#ifndef DYNAMICSLAB_MATRIXUTILITIES_H
#define DYNAMICSLAB_MATRIXUTILITIES_H

#include <Eigen/Dense>

class MatrixUtilities {
public:
    Eigen::Matrix<double, 3, 4> transformationMatrixL(Eigen::Vector4d transformationMatrix);
    Eigen::Matrix<double, 3, 4> transformationMatrixG(Eigen::Vector4d transformationMatrix);
};


#endif //DYNAMICSLAB_MATRIXUTILITIES_H
