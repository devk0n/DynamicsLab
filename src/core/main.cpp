#include <iostream>
#include <memory>

#include "application.h"
#include "dynamics.h"

bool debug = false;

int main() {

    if (debug) {

        auto body1 = std::make_shared<RigidBody>(
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector4d(1, 0, 0, 0),
                Eigen::Matrix3d::Identity() * 10,
                Eigen::Matrix3d::Identity() * 60);
        auto body2 = std::make_shared<RigidBody>(
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector4d(1, 0, 0, 0),
                Eigen::Matrix3d::Identity() * 10,
                Eigen::Matrix3d::Identity() * 60);
        auto body3 = std::make_shared<RigidBody>(
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector4d(1, 0, 0, 0),
                Eigen::Matrix3d::Identity() * 10,
                Eigen::Matrix3d::Identity() * 60);

        Dynamics dynamics;
        dynamics.addBody(body1);
        dynamics.addBody(body2);
        dynamics.addBody(body3);
        dynamics.step(0.01);

        dynamics.debug();

    } else {
        try {
            Application app(1920, 1080, "DynamicsLab");
            app.run();
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
