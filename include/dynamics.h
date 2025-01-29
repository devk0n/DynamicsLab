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
    explicit Dynamics();

    void addBody(const std::shared_ptr<RigidBody>& body);
    void step(double deltaTime);

    void debug();

    Eigen::MatrixXd getSystemMassInertiaMatrix();
    Eigen::MatrixXd getQuaternionConstraintMatrix();
    Eigen::MatrixXd getGeneralizedCoordinates();
    Eigen::MatrixXd getGeneralizedVelocities();
    Eigen::MatrixXd getGeneralizedAccelerations();
    Eigen::MatrixXd getVelocityDependentTerm();

    Eigen::MatrixXd getAZeros();

    Eigen::MatrixXd getQuaternionNormSquared();

    Eigen::MatrixXd getGeneralizedExternalForces();
    Eigen::MatrixXd getMatrixA();
    Eigen::MatrixXd getMatrixB();
    Eigen::MatrixXd getMatrixX();
    
    int getBodyCount();
    std::shared_ptr<RigidBody> &getBody(int index);


private:
    std::vector<std::shared_ptr<RigidBody>> m_Bodies;

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

    void initializeSize();
    void initializeContent();
};


#endif //DYNAMICSLAB_DYNAMICS_H
