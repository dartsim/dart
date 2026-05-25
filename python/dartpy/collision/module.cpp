#include "collision/module.hpp"

#include "collision/collision_detector.hpp"
#include "collision/collision_group.hpp"
#include "collision/collision_option.hpp"
#include "collision/collision_result.hpp"
#include "collision/continuous_collision.hpp"
#include "collision/raycast.hpp"

namespace dart::python_nb {

void defCollisionModule(nanobind::module_& m)
{
  defCollisionOption(m);
  defCollisionResult(m);
  defCollisionDetector(m);
  defRaycast(m);
  defContinuousCollision(m);
  defCollisionGroup(m);
}

} // namespace dart::python_nb
