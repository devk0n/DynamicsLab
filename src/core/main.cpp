#include <iostream>

#include "application.h"

int WIDTH = 1920;
int HEIGHT = 1080;

int main() {

    try {
        Application app(WIDTH, HEIGHT, "DynamicsLab");
        app.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
