#include "collision/collision_detector.hpp"

#include "collision/collision_group.hpp"
#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#ifdef HAVE_BULLET
  #include "dart/collision/bullet/BulletCollisionDetector.hpp"
#endif
#ifdef HAVE_ODE
  #include "dart/collision/ode/OdeCollisionDetector.hpp"
#endif

namespace nb = nanobind;

namespace dart::python_nb {

void defCollisionDetector(nb::module_& m)
{
  using CollisionDetector = dart::collision::CollisionDetector;

  nb::class_<CollisionDetector>(m, "CollisionDetector")
      .def(
          "getType",
          [](const CollisionDetector& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal)
      .def("createCollisionGroup", [](CollisionDetector& self) {
        return self.createCollisionGroupAsSharedPtr();
      });

  nb::class_<dart::collision::FCLCollisionDetector, CollisionDetector>(
      m, "FCLCollisionDetector")
      .def(nb::new_([](nb::handle)
                    -> std::shared_ptr<dart::collision::FCLCollisionDetector> {
            return dart::collision::FCLCollisionDetector::create();
          }))
      .def_static(
          "getStaticType",
          []() -> const std::string& {
            return dart::collision::FCLCollisionDetector::getStaticType();
          },
          nb::rv_policy::reference_internal);

  nb::class_<dart::collision::DARTCollisionDetector, CollisionDetector>(
      m, "DARTCollisionDetector")
      .def(nb::new_(
          [](nb::handle)
              -> std::shared_ptr<dart::collision::DARTCollisionDetector> {
            return dart::collision::DARTCollisionDetector::create();
          }))
      .def_static(
          "getStaticType",
          []() -> const std::string& {
            return dart::collision::DARTCollisionDetector::getStaticType();
          },
          nb::rv_policy::reference_internal);

#ifdef HAVE_BULLET
  nb::class_<dart::collision::BulletCollisionDetector, CollisionDetector>(
      m, "BulletCollisionDetector")
      .def(nb::new_(
          [](nb::handle)
              -> std::shared_ptr<dart::collision::BulletCollisionDetector> {
            return dart::collision::BulletCollisionDetector::create();
          }))
      .def_static(
          "getStaticType",
          []() -> const std::string& {
            return dart::collision::BulletCollisionDetector::getStaticType();
          },
          nb::rv_policy::reference_internal);
#endif

#ifdef HAVE_ODE
  nb::class_<dart::collision::OdeCollisionDetector, CollisionDetector>(
      m, "OdeCollisionDetector")
      .def(nb::new_(
          [](nb::handle)
              -> std::shared_ptr<dart::collision::OdeCollisionDetector> {
            return dart::collision::OdeCollisionDetector::create();
          }))
      .def_static(
          "getStaticType",
          []() -> const std::string& {
            return dart::collision::OdeCollisionDetector::getStaticType();
          },
          nb::rv_policy::reference_internal);
#endif
}

} // namespace dart::python_nb
