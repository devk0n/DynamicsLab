#pragma once
#include <Eigen/Dense>
#include <vector>

class Simulation {
public:
    Simulation();
    void run();
private:
    void initialize();
    void step(double timeStep);
    void integrate(double timeStep);

    Eigen::VectorXd q1, q1d;
    double m1, l1, h1, w1;
    Eigen::Matrix3d N1;
    Eigen::Matrix3d J1m;
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector4d> orientations;
};