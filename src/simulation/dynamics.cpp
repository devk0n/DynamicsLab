#include <iostream>
#include "dynamics.h"

Dynamics::Dynamics(int b) {
    m_SystemMassInertiaMatrix.resize(7 * b, 7 * b);
    m_QuaternionConstraintMatrix.resize(1 * b, 7 * b);


    m_GeneralizedCoordinates.resize(7 * b, 1);
    m_GeneralizedVelocities.resize(7 * b, 1);
    m_GeneralizedAccelerations.resize(7 * b, 1);
    m_VelocityDependentTerm.resize(7 * b, 1);
    m_QuaternionNormSquared.resize(b, 1);
    m_GeneralizedExternalForces.resize(7 * b, 1);

    m_A.resize(7 * b + b, 7 * b + b);
    m_AZeros.resize(b, b); // Unconstrained

    m_B.resize(7 * b + b, 1);
    m_BZeros.resize(b, 1);

    m_X.resize(7 * b + b, 1);

    m_SystemMassInertiaMatrix.setZero();
    m_QuaternionConstraintMatrix.setZero();
    m_AZeros.setZero();
    m_GeneralizedCoordinates.setZero();
    m_GeneralizedVelocities.setZero();
    m_GeneralizedAccelerations.setZero();
    m_VelocityDependentTerm.setZero();
    m_QuaternionNormSquared.setZero();
    m_GeneralizedExternalForces.setZero();

    m_A.setZero();
    m_B.setZero();
    m_X.setZero();

    std::cout << "Dynamics Initialized" << std::endl;
    std::cout << "System Mass Inertia Matrix: " << std::endl;
    std::cout << m_SystemMassInertiaMatrix << std::endl;
    std::cout << "Quaternion Constraint Matrix: " << std::endl;
    std::cout << m_QuaternionConstraintMatrix << std::endl;

    std::cout << "AZeros: " << std::endl;
    std::cout << m_AZeros << std::endl;

    std::cout << "Generalized Coordinates: " << std::endl;
    std::cout << m_GeneralizedCoordinates << std::endl;
    std::cout << "Generalized Velocities: " << std::endl;
    std::cout << m_GeneralizedVelocities << std::endl;
    std::cout << "Generalized Accelerations: " << std::endl;
    std::cout << m_GeneralizedAccelerations << std::endl;

    std::cout << "Velocity Dependent Term: " << std::endl;
    std::cout << m_VelocityDependentTerm << std::endl;
    std::cout << "Quaternion Norm Squared: " << std::endl;
    std::cout << m_QuaternionNormSquared << std::endl;
    std::cout << "Generalized External Forces: " << std::endl;
    std::cout << m_GeneralizedExternalForces << std::endl;

    std::cout << "A: " << m_A.rows() << " " << m_A.cols() << std::endl;
    std::cout << m_A << std::endl;
    std::cout << "B: " << m_B.rows() << " " << m_B.cols() << std::endl;
    std::cout << m_B << std::endl;
    std::cout << "X: " << m_X.rows() << " " << m_X.cols() << std::endl;
    std::cout << m_X << std::endl;
}

void Dynamics::addBody() {

}
/*
void Dynamics::step() {
    Vector3d r1 = q1.head<3>();
    Vector4d p1 = q1.segment<4>(3);
    // Vector3d r1d = q1d.head<3>();
    Vector4d p1d = q1d.segment<4>(3);

    // Inertia and Constraint Matrices
    Matrix<double, 3, 4> L1 = lMatrix(p1);
    Matrix4d J1s = 4 * L1.transpose() * J1m * L1;
    MatrixXd Ms(7, 7);
    Ms.setZero();
    Ms.topLeftCorner<3, 3>() = N1;
    Ms.bottomRightCorner<4, 4>() = J1s;

    Matrix<double, 3, 7> B;
    B.setZero();
    B.topLeftCorner<3, 3>() = -Matrix3d::Identity();
    B.rightCols<4>() = -2 * gMatrix(p1) * skewN(s1Am);

    Matrix<double, 3, 4> L1d = lMatrix(p1d);
    Vector3d gamma = -(-2 * gMatrix(p1d) * L1d.transpose() * s1Am);

    Matrix<double, 1, 7> P;
    P.setZero();
    P.rightCols<4>() = p1.transpose();

    MatrixXd A(11, 11);
    A.setZero();
    A.topLeftCorner<7, 7>() = Ms;
    A.block<1, 7>(7, 0) = P;
    A.block<7, 1>(0, 7) = P.transpose();
    A.block<3, 7>(8, 0) = B;
    A.block<7, 3>(0, 8) = B.transpose();

    VectorXd bs(7);
    bs.setZero();
    bs.tail<4>() = 2 * (4 * L1d.transpose() * J1m * L1 * p1d);

    double c = p1d.squaredNorm();
    VectorXd Bv(11);
    Bv.setZero();
    Bv.segment<7>(0) = bs;
    Bv(7) = c;

    Vector3d f1(0.0, 0.0, m1 * g);
    Vector4d n1s = Vector4d::Zero();

    VectorXd gs(11);
    gs << f1, n1s, 0, gamma;

    VectorXd x = A.partialPivLu().solve(gs - Bv);
    VectorXd q1dd = x.segment<7>(0);
}

 */