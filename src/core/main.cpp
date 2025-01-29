#include <iostream>

#include "application.h"
#include "dynamics.h"

bool debug = true;

int main() {

    if (debug) {

        Dynamics dynamics(1);

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
