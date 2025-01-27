#include <iostream>
#include <chrono>

#include "simulation.h"

void Simulation::run() {
    auto start = std::chrono::steady_clock::now();

    while (taskActive) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start;

        if (elapsed.count() >= timeStep) {
            time += timeStep;
            updateDebug();
            // Move forward the start time by fixed timeStep to account for missed frames
            start += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(timeStep)
            );
        }
    }
}


void Simulation::stop() {
    taskActive = false;
}

void Simulation::updateDebug() {
    graphics->time = time;
}
