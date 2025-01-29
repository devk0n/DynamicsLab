#include <iostream>

#include "application.h"
#include "rigid_body.h"

bool debug = true;

int main() {

    if (debug) {

        Eigen::Vector3d position(7, 6, 7);
        Eigen::Vector4d orientation(1, 0, 0, 0);
        double mass = 6277;
        Eigen::Matrix3d inertiaTensor = (Eigen::Matrix3d() << 0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.3).finished();

        RigidBody body(position, orientation, mass, inertiaTensor);

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
