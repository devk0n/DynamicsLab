#include <iostream>

#include "simulation.h"

void Simulation::run() {
    while (taskActive) {
        time += timeStep;
        std::cout << time << std::endl;
        updateDebug();
    }
}

void Simulation::stop() {
    taskActive = false;
}

void Simulation::updateDebug() {
    graphics->time = time;
}
