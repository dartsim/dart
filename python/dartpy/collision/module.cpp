#include "collision/module.hpp"

#include "collision/collision_detector.hpp"
#include "collision/collision_group.hpp"
#include "collision/collision_object.hpp"
#include "collision/collision_option.hpp"
#include "collision/collision_result.hpp"
#include "collision/contact.hpp"
#include "collision/raycast.hpp"

namespace dart::python_nb {

void defCollisionModule(nanobind::module_& m)
{
  defCollisionObject(m);
  defContact(m);
  defCollisionOption(m);
  defCollisionResult(m);
  defRaycast(m);
  defCollisionDetector(m);
  defCollisionGroup(m);
}

} // namespace dart::python_nb
