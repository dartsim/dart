#pragma once

namespace dart::simulation {
class World;
}

namespace dartnb {
// The GIL-free stepping binding will enable this shared accessor guard.
inline void checkWorldBusy(const dart::simulation::World& world)
{
  static_cast<void>(world);
}
} // namespace dartnb
