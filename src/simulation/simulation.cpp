#include "simulation.h"


Simulation::Simulation(double timeStep) {};


void Simulation::addRigidBody(const RigidBody& body){

}


void Simulation::update(){

}


const std::vector<RigidBody>& Simulation::getRigidBodies() const {
    return m_RigidBodies;
}
