#ifndef FORCE_GENERATOR_H
#define FORCE_GENERATOR_H

namespace Proton {
class ForceGenerator {
public:

  virtual ~ForceGenerator() = default;
  virtual void apply(double dt) = 0;

};
} // Proton

#endif // FORCE_GENERATOR_H
