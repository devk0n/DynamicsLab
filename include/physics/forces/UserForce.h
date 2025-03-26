#ifndef USER_FORCE_H
#define USER_FORCE_H

#include "Proton.h"
#include "Body.h"
#include "ForceGenerator.h"
#include <unordered_map>

namespace Proton {

class UserForce final : public ForceGenerator {
public:
  void addBody(Body* body) {
    m_targets.emplace(body->getID(), body);
  }

  void apply(double dt) override;

private:
  std::unordered_map<UniqueID, Body*> m_targets;

};
} // Proton

#endif // USER_FORCE_H
