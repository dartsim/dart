#include "collision/collision_detector.hpp"

#include "collision/collision_group.hpp"
#include "dart/collision/collision_detector.hpp"
#include "dart/collision/dart/dart_collision_detector.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defCollisionDetector(nb::module_& m)
{
  using CollisionDetector = dart::collision::CollisionDetector;

  nb::class_<CollisionDetector>(m, "CollisionDetector")
      .def(
          "getType",
          [](const CollisionDetector& self) {
            return std::string(self.getTypeView());
          })
      .def("createCollisionGroup", [](CollisionDetector& self) {
        return self.createCollisionGroupAsSharedPtr();
      });

  nb::class_<dart::collision::DartCollisionDetector, CollisionDetector>(
      m, "DartCollisionDetector")
      .def(
          nb::new_(
              []() -> std::shared_ptr<dart::collision::DartCollisionDetector> {
                return dart::collision::DartCollisionDetector::create();
              }))
      .def_static("getStaticType", []() {
        return std::string(
            dart::collision::DartCollisionDetector::getStaticType());
      });

  // Dartpy exposes the clean DART 7 API directly. C++ retains deprecated
  // legacy detector facades for downstream source compatibility.
}

} // namespace dart::python_nb
