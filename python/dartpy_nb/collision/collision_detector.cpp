#include "collision/collision_detector.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "collision/collision_group.hpp"

#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defCollisionDetector(nb::module_& m)
{
  using CollisionDetector = dart::collision::CollisionDetector;

  nb::class_<CollisionDetector, std::shared_ptr<CollisionDetector>>(m, "CollisionDetector")
      .def("getType",
          [](const CollisionDetector& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal)
      .def("createCollisionGroup",
          [](CollisionDetector& self) {
            return self.createCollisionGroupAsSharedPtr();
          });

  nb::class_<dart::collision::FCLCollisionDetector, CollisionDetector, std::shared_ptr<dart::collision::FCLCollisionDetector>>(m, "FCLCollisionDetector")
      .def(nb::init<>())
      .def_static("getStaticType",
          []() -> const std::string& {
            return dart::collision::FCLCollisionDetector::getStaticType();
          },
          nb::rv_policy::reference_internal);

  nb::class_<dart::collision::DARTCollisionDetector, CollisionDetector, std::shared_ptr<dart::collision::DARTCollisionDetector>>(m, "DARTCollisionDetector")
      .def(nb::init<>())
      .def_static("getStaticType",
          []() -> const std::string& {
            return dart::collision::DARTCollisionDetector::getStaticType();
          },
          nb::rv_policy::reference_internal);
}

} // namespace dart::python_nb
