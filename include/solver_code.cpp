#include <iostream>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

// Matrix functions
Matrix<double, 3, 4> gMatrix(const Vector4d &p) {
    Matrix<double, 3, 4> G;
    G << -p(1), p(0), -p(3), p(2),
         -p(2), p(3), p(0), -p(1),
         -p(3), -p(2), p(1), p(0);
    return G;
}

Matrix<double, 3, 4> lMatrix(const Vector4d &p) {
    Matrix<double, 3, 4> L;
    L << -p(1), p(0), p(3), -p(2),
         -p(2), -p(3), p(0), p(1),
         -p(3), p(2), -p(1), p(0);
    return L;
}

Matrix4d skewN(const Vector3d &a) {
    Matrix4d Sn = Matrix4d::Zero();
    Sn.block<1, 3>(0, 1) = -a.transpose();
    Sn.block<3, 1>(1, 0) = a;
    Sn.block<3, 3>(1, 1) = -Matrix3d((Matrix3d() << 0, -a(2), a(1),
                                                    a(2), 0, -a(0),
                                                   -a(1), a(0), 0).finished());
    return Sn;
}

int main() {
    // Constants
    constexpr double m1 = 10.0, l1 = 2.0, h1 = 0.7, w1 = 0.7;

    constexpr double Ix = (1.0 / 12.0) * m1 * (h1 * h1 + w1 * w1);
    constexpr double Iy = (1.0 / 12.0) * m1 * (l1 * l1 + w1 * w1);
    constexpr double Iz = (1.0 / 12.0) * m1 * (l1 * l1 + h1 * h1);

    constexpr double totalTime = 10.0, timeStep = 0.001;

    int steps = static_cast<int>(totalTime / timeStep);

    // Initial Conditions
    VectorXd q1(7), q1d(7);
    q1 << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    q1d.setZero();

    // Vector3d s0A = Vector3d::Zero();
    Vector3d s1Am(-l1/2, 0.0, 0.0);  // s1Bm(L2, 0.0, 0.0);

    // Mass and Inertia matrix
    Matrix3d N1 = (Matrix3d() << m1, 0, 0, 0, m1, 0, 0, 0, m1).finished();
    Matrix3d J1m = (Matrix3d() << Ix, 0, 0, 0, Iy, 0, 0, 0, Iz).finished();

    // Storage
    vector<Vector3d> positions(steps);
    vector<Vector4d> orientations(steps);

    for (int step = 0; step < steps; ++step) {
        constexpr double g = 9.81;
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

        // Integration
        q1d += q1dd * timeStep;
        q1 += q1d * timeStep;

        // Normalize quaternion
        p1 = q1.segment<4>(3);
        q1.segment<4>(3) = p1 / p1.norm();

        // Store results
        positions[step] = r1;
        orientations[step] = q1.segment<4>(3);

    }

    // Print Final Results
    for (int i = 0; i < steps; i += steps / 100) {
        cout << "Time Step: " << i * timeStep << endl;
        cout << "Position: " << positions[i].transpose() << endl;
        cout << "Orientation: " << orientations[i].transpose() << endl;
        cout << "--------------------------" << endl;
    }

    return 0;
}