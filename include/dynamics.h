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

    Eigen::VectorXd getGeneralizedCoordinates();
    Eigen::VectorXd getGeneralizedVelocities();
    Eigen::VectorXd getGeneralizedAccelerations();

    Eigen::VectorXd getVelocityDependentTerm();

    Eigen::VectorXd getQuaternionNormSquared();

    Eigen::VectorXd getGeneralizedExternalForces();

    Eigen::MatrixXd getMatrixA();
    Eigen::VectorXd getMatrixB();
    Eigen::VectorXd getMatrixX();
    
    int getBodyCount();
    std::shared_ptr<RigidBody> &getBody(int index);


private:
    std::vector<std::shared_ptr<RigidBody>> m_Bodies;

    Eigen::MatrixXd m_SystemMassInertiaMatrix;
    Eigen::MatrixXd m_QuaternionConstraintMatrix;

    Eigen::VectorXd m_GeneralizedCoordinates;
    Eigen::VectorXd m_GeneralizedVelocities;
    Eigen::VectorXd m_GeneralizedAccelerations;

    Eigen::VectorXd m_VelocityDependentTerm;

    Eigen::VectorXd m_QuaternionNormSquared;
    Eigen::VectorXd m_GeneralizedExternalForces;

    Eigen::VectorXd m_ExternalForces;

    Eigen::MatrixXd m_A;
    Eigen::VectorXd m_B;
    Eigen::VectorXd m_X;

    void initializeSize();
    void initializeContent();
};


#endif //DYNAMICSLAB_DYNAMICS_H
