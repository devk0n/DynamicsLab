#include <iostream>
#include "application.h"

int main() {

    try {
        Application app(1920, 1080, "Dynamics Lab");
        app.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
