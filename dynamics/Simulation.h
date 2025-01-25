#ifndef SIMULATION_H
#define SIMULATION_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

struct SimulationResult {
    std::vector<Vector3d> positions;
    std::vector<Vector4d> orientations;
    std::uint16_t calculationTime;
};

void stepSimulation(double timeStep, VectorXd &q1, VectorXd &q1d);

#endif // SIMULATION_H
