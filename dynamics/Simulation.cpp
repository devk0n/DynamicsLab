#include "simulation.h"

void stepSimulation(double timeStep, VectorXd &q1, VectorXd &q1d) {
    constexpr double m1 = 10.0, l1 = 2.0, h1 = 0.7, w1 = 0.7;
    constexpr double Ix = (1.0 / 12.0) * m1 * (h1 * h1 + w1 * w1);
    constexpr double g = 9.81;

    Vector3d gravity(0.0, 0.0, -g);
    q1d.head<3>() += gravity * timeStep;
    q1.head<3>() += q1d.head<3>() * timeStep;

    // Normalize quaternion to avoid numerical drift
    q1.segment<4>(3).normalize();
}
