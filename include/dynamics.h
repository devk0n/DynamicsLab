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

using namespace Eigen;

class Dynamics {
public:
    explicit Dynamics();

    void addBody(const std::shared_ptr<RigidBody>& body);
    void step();

    void debug();

    MatrixXd getSystemMassInertiaMatrix();      // M*
    MatrixXd getQuaternionConstraintMatrix();   // P

    VectorXd getGeneralizedCoordinates();       // q
    VectorXd getGeneralizedVelocities();        // qd
    VectorXd getGeneralizedAccelerations();     // qdd

    VectorXd getVelocityDependentTerm();        // b*

    VectorXd getQuaternionNormSquared();        // c

    VectorXd getGeneralizedExternalForces();    // g*

    MatrixXd getMatrixA();
    VectorXd getMatrixB();
    VectorXd getMatrixX();
    
    int getBodyCount();
    std::shared_ptr<RigidBody> &getBody(int index);

    void setExternalTorques(Vector3d externalTorques);

    void startSimulation();
    void stopSimulation();
    void resetSimulation();

    void setExternalForces(Vector3d matrix);

    void setStepTime(double stepTime);

private:
    double m_stepTime = 0.0002;

    std::vector<std::shared_ptr<RigidBody>> m_Bodies;

    MatrixXd m_SystemMassInertiaMatrix;
    MatrixXd m_QuaternionConstraintMatrix;

    VectorXd m_GeneralizedCoordinates;
    VectorXd m_GeneralizedVelocities;
    VectorXd m_GeneralizedAccelerations;

    VectorXd m_VelocityDependentTerm;

    VectorXd m_QuaternionNormSquared;
    VectorXd m_GeneralizedExternalForces;

    Vector3d m_ExternalForces;
    Vector3d m_ExternalTorques;

    MatrixXd m_A;
    VectorXd m_B;
    VectorXd m_X;

    bool m_isSimulationRunning = false;

    void initializeSize();
    void initializeContent();
};


#endif //DYNAMICSLAB_DYNAMICS_H
