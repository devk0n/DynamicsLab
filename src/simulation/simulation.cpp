//
// Created by devkon on 28/01/2025.
//

#include "simulation.h"

Simulation::Simulation(double timeStep) {};

void Simulation::addRigidBody(const RigidBody& body){

}

void Simulation::update(){

}

const std::vector<RigidBody>& Simulation::getRigidBodies() const {
    return m_RigidBodies;
}
