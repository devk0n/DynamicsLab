#ifndef DYNAMICSLAB_SIMULATION_H
#define DYNAMICSLAB_SIMULATION_H

#include "rigid_body.h"

class Simulation {
public:
    Simulation(double timeStep);

    void addRigidBody(const RigidBody& body);
    void update();
    const std::vector<RigidBody>& getRigidBodies() const;

private:
    double m_TimeStep;
    std::vector<RigidBody> m_RigidBodies;
};


#endif //DYNAMICSLAB_SIMULATION_H
