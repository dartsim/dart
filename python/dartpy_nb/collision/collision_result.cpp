#include "collision/collision_result.hpp"

#include <nanobind/nanobind.h>

#include "dart/collision/CollisionResult.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defCollisionResult(nb::module_& m)
{
  using CollisionResult = dart::collision::CollisionResult;

  nb::class_<CollisionResult>(m, "CollisionResult")
      .def(nb::init<>())
      .def("clear", &CollisionResult::clear)
      .def("isCollision", &CollisionResult::isCollision)
      .def("getNumContacts", &CollisionResult::getNumContacts);
}

} // namespace dart::python_nb
