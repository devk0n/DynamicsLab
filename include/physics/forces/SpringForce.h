#ifndef SPRING_FORCE_GENERATOR_H
#define SPRING_FORCE_GENERATOR_H

#include "ForceGenerator.h"

namespace Proton {

class SpringForceGenerator : public ForceGenerator {
public:
  SpringForceGenerator();

  void apply(double dt) override;

private:

};
} // Proton

#endif // SPRING_FORCE_GENERATOR_H
