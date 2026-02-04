#include "collision/collision_result.hpp"

#include "common/repr.hpp"
#include "dart/collision/collision_result.hpp"

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defCollisionResult(nb::module_& m)
{
  using CollisionResult = dart::collision::CollisionResult;

  nb::class_<CollisionResult>(m, "CollisionResult")
      .def(nb::init<>())
      .def("clear", &CollisionResult::clear)
      .def("isCollision", &CollisionResult::isCollision)
      .def("getNumContacts", &CollisionResult::getNumContacts)
      .def("__repr__", [](const CollisionResult& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("is_collision", repr_bool(self.isCollision()));
        fields.emplace_back("contacts", std::to_string(self.getNumContacts()));
        return format_repr("CollisionResult", fields);
      });
}

} // namespace dart::python_nb
