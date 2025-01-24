#include "Simulation.h"
#include "MatrixUtils.h"
#include <iostream>

Simulation::Simulation()
    : q1(7), q1d(7), positions(), orientations() {
    initialize();
}

void Simulation::initialize() {
    m1 = 10.0; l1 = 2.0; h1 = 0.7; w1 = 0.7;
    N1 = Eigen::Matrix3d::Identity() * m1;
    J1m << (1.0 / 12.0) * m1 * (h1 * h1 + w1 * w1), 0, 0,
           0, (1.0 / 12.0) * m1 * (l1 * l1 + w1 * w1), 0,
           0, 0, (1.0 / 12.0) * m1 * (l1 * l1 + h1 * h1);

    q1 << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    q1d.setZero();
}

void Simulation::step(double timeStep) {
    using namespace MatrixUtils;
    Eigen::Vector3d s1Am(-l1 / 2, 0.0, 0.0);
}

void Simulation::run() {
    constexpr double totalTime = 10.0, timeStep = 0.001;
    int steps = static_cast<int>(totalTime / timeStep);

    for (int step = 0; step < steps; ++step) {
        step(timeStep);
    }

    for (int i = 0; i < steps; i += steps / 100) {
        std::cout << "Position: " << positions[i].transpose() << "\n"
                  << "Orientation: " << orientations[i].transpose() << "\n";
    }
}