#include "collision/collision_object.hpp"

#include "common/type_casters.hpp"
#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/dynamics/ShapeFrame.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defCollisionObject(nb::module_& m)
{
  using CollisionObject = dart::collision::CollisionObject;

  nb::class_<CollisionObject>(m, "CollisionObject")
      .def(
          "getCollisionDetector",
          [](CollisionObject& self) { return self.getCollisionDetector(); },
          nb::rv_policy::reference_internal)
      .def(
          "getShapeFrame",
          [](CollisionObject& self) { return self.getShapeFrame(); },
          nb::rv_policy::reference_internal)
      .def(
          "getShape",
          [](CollisionObject& self) -> dart::dynamics::ConstShapePtr {
            return self.getShape();
          })
      .def("getTransform", [](const CollisionObject& self) {
        return self.getTransform();
      });
}

} // namespace dart::python_nb
