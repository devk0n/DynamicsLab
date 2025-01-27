#ifndef DYNAMICSLAB_SIMULATION_H
#define DYNAMICSLAB_SIMULATION_H

#include "graphics_manager.h"

class Simulation {
public:
    explicit Simulation(GraphicsManager* graphics)
        : graphics(graphics) {};

    void run();
    void stop();

private:
    GraphicsManager* graphics;

    bool taskActive = true;
    double time = 0.0;
    double timeStep = 0.0001;

    void updateDebug();
};


#endif //DYNAMICSLAB_SIMULATION_H
