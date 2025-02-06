#include <iostream>
#include "core/Application.h"

int main() {
    Application app;

    // Initialize subsystems
    if (!app.initialize()) {
        std::cerr << "Failed to initialize application. \n";
        return EXIT_FAILURE;
    }

    // Run the main loop
    app.run();

    // Application destructor will call shutdown, or you can call it manually
    return EXIT_SUCCESS;
}
