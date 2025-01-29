#include <iostream>

#include "application.h"
#include "dynamics.h"

bool debug = true;

int main() {

    if (debug) {



        RigidBody body(
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector4d(1, 0, 0, 0),
                Eigen::Matrix3d::Identity() * 10,
                Eigen::Matrix3d::Identity() * 60);

        // Dynamics dynamics(1);

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
