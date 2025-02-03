#include <iostream>

#include "../application/application.h"

int main() {

    try {
        Application app(1920, 1080, "DynamicsLab");
        app.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
