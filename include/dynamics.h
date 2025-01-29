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

    QuaternionConstraintRHS         (c)
    GeneralizedExternalForces       (g*)
 */


class Dynamics {
public:

private:
    Eigen::MatrixXd m_SystemMassInertiaMatrix;
    Eigen::MatrixXd m_QuaternionConstraintMatrix;

    Eigen::MatrixXd m_GeneralizedCoordinates;
    Eigen::MatrixXd m_GeneralizedVelocities;
    Eigen::MatrixXd m_GeneralizedAccelerations;

    Eigen::MatrixXd m_VelocityDependentTerm;

    Eigen::MatrixXd m_QuaternionConstraintRHS;
    Eigen::MatrixXd m_GeneralizedExternalForces;

    Eigen::VectorXd m_VelocityTerm;
    Eigen::VectorXd externalForces;
};


#endif //DYNAMICSLAB_DYNAMICS_H
