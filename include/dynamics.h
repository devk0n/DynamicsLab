#ifndef DYNAMICSLAB_DYNAMICS_H
#define DYNAMICSLAB_DYNAMICS_H

#include <Eigen/Dense>
#include <memory>

#include "rigid_body.h"

/*
    SystemMassInertiaMatrix         (M*)
    QuaternionConstraintMatrix      (P)

    GeneralizedCoordinates          (q)
    GeneralizedVelocities           (qd)
    GeneralizedAccelerations        (qdd)

    VelocityDependentTerm           (b*)

    QuaternionNormSquared           (c)
    GeneralizedExternalForces       (g*)
 */


class Dynamics {
public:
    explicit Dynamics(int bodies);

    void addBody();
private:
    Eigen::MatrixXd m_SystemMassInertiaMatrix;
    Eigen::MatrixXd m_QuaternionConstraintMatrix;

    Eigen::MatrixXd m_GeneralizedCoordinates;
    Eigen::MatrixXd m_GeneralizedVelocities;
    Eigen::MatrixXd m_GeneralizedAccelerations;

    Eigen::MatrixXd m_VelocityDependentTerm;

    Eigen::VectorXd m_QuaternionNormSquared;
    Eigen::MatrixXd m_GeneralizedExternalForces;

    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_B;
    Eigen::MatrixXd m_X;

    Eigen::MatrixXd m_AZeros;
    Eigen::MatrixXd m_BZeros;

};


#endif //DYNAMICSLAB_DYNAMICS_H
