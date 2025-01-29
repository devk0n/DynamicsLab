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

    Eigen::MatrixXd getMatrixA();
    size_t getBodyCount() const;
    const std::shared_ptr<RigidBody> &getBody(size_t index) const;


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

    void initialize();

};


#endif //DYNAMICSLAB_DYNAMICS_H
