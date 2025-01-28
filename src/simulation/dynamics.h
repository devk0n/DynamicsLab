#ifndef DYNAMICSLAB_DYNAMICS_H
#define DYNAMICSLAB_DYNAMICS_H

#include <Eigen/Dense>

class Dynamics {
public:
    Dynamics();
    ~Dynamics();

    void run();

private:

    // Ms size is 7 * "b" bodies
    // One body example
    Eigen::MatrixXd Ms;

    Eigen::Matrix<double, 3, 4> gMatrix(const Eigen::Vector4d &p);
    Eigen::Matrix<double, 3, 4> lMatrix(const Eigen::Vector4d &p);
    Eigen::Matrix4d skewN(const Eigen::Vector3d &a);
};


#endif //DYNAMICSLAB_DYNAMICS_H
