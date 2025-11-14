#include "collision/module.hpp"

#include "collision/collision_detector.hpp"
#include "collision/collision_group.hpp"
#include "collision/collision_option.hpp"
#include "collision/collision_result.hpp"
#include "collision/raycast.hpp"

namespace dart::python_nb {

void defCollisionModule(nanobind::module_& m)
{
  auto collision = m.def_submodule("collision");

  defCollisionOption(collision);
  defCollisionResult(collision);
  defRaycast(collision);
  defCollisionDetector(collision);
  defCollisionGroup(collision);
}

} // namespace dart::python_nb
