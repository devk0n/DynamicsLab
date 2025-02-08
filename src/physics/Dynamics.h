#ifndef DYNAMICSLAB_DYNAMICS_H
#define DYNAMICSLAB_DYNAMICS_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "RigidBody.h"
#include "graphics/Renderer.h"

class Dynamics {
public:
    Dynamics(Renderer* renderer, double stepSize);

    void initialize();

    void addRigidBody(std::unique_ptr<RigidBody> rigidBody);

    void step();

    VectorXd getGeneralizedCoordinates();
    VectorXd getGeneralizedVelocities();
    VectorXd getGeneralizedAccelerations();

    VectorXd getGeneralizedExternalForces();

    MatrixXd getMatrixA();
    VectorXd getVectorB();
    VectorXd getVectorX();

    void start() { m_running = true; }
    void stop() { m_running = false; }

    void setExternalForce(Matrix<double, 3, 1> matrix);

private:
    std::vector<std::unique_ptr<RigidBody>> m_rigidBodies;

    Renderer *m_renderer;

    double m_stepSize;
    int m_bodyCount{};

    bool m_running;

    MatrixXd m_systemMassInertiaMatrix;
    MatrixXd m_quaternionConstraintMatrix;

    VectorXd m_generalizedCoordinates;
    VectorXd m_generalizedVelocities;
    VectorXd m_generalizedAccelerations;

    VectorXd m_velocityDependentTerm;

    VectorXd m_quaternionNormSquared;
    VectorXd m_generalizedExternalForces;

    Vector3d m_externalForces;
    Vector4d m_externalTorques;

    MatrixXd m_matrixA;
    VectorXd m_vectorB;
    VectorXd m_vectorX;

    static Matrix<double, 3, 4> transformationMatrixL(Vector4d transformationMatrix);


};


#endif
