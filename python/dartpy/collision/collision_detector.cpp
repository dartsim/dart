#include "collision/collision_detector.hpp"

#include "collision/collision_group.hpp"
#include "dart/collision/collision_detector.hpp"
#include "dart/collision/dart/dart_collision_detector.hpp"
#if DART_HAVE_FCL
  #include "dart/collision/fcl/fcl_collision_detector.hpp"
#endif
#ifdef DART_HAS_EXPERIMENTAL_COLLISION
  #include "dart/collision/experimental_backend/experimental_collision_detector.hpp"
#endif

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#if DART_HAVE_BULLET
  #include "dart/collision/bullet/bullet_collision_detector.hpp"
#endif
#if DART_HAVE_ODE
  #include "dart/collision/ode/ode_collision_detector.hpp"
#endif

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

#if DART_HAVE_FCL
  nb::class_<dart::collision::FCLCollisionDetector, CollisionDetector>(
      m, "FCLCollisionDetector")
      .def(
          nb::new_(
              []() -> std::shared_ptr<dart::collision::FCLCollisionDetector> {
                return dart::collision::FCLCollisionDetector::create();
              }))
      .def_static("getStaticType", []() {
        return std::string(
            dart::collision::FCLCollisionDetector::getStaticType());
      });
#endif

  nb::class_<dart::collision::DARTCollisionDetector, CollisionDetector>(
      m, "DARTCollisionDetector")
      .def(
          nb::new_(
              []() -> std::shared_ptr<dart::collision::DARTCollisionDetector> {
                return dart::collision::DARTCollisionDetector::create();
              }))
      .def_static("getStaticType", []() {
        return std::string(
            dart::collision::DARTCollisionDetector::getStaticType());
      });

#ifdef DART_HAS_EXPERIMENTAL_COLLISION
  nb::class_<dart::collision::ExperimentalCollisionDetector, CollisionDetector>(
      m, "ExperimentalCollisionDetector")
      .def(
          nb::new_(
              []() -> std::shared_ptr<
                       dart::collision::ExperimentalCollisionDetector> {
                return dart::collision::ExperimentalCollisionDetector::create();
              }))
      .def_static("getStaticType", []() {
        return std::string(
            dart::collision::ExperimentalCollisionDetector::getStaticType());
      });
#endif

#if DART_HAVE_BULLET
  nb::class_<dart::collision::BulletCollisionDetector, CollisionDetector>(
      m, "BulletCollisionDetector")
      .def(
          nb::new_(
              []()
                  -> std::shared_ptr<dart::collision::BulletCollisionDetector> {
                return dart::collision::BulletCollisionDetector::create();
              }))
      .def_static("getStaticType", []() {
        return std::string(
            dart::collision::BulletCollisionDetector::getStaticType());
      });
#endif

#if DART_HAVE_ODE
  nb::class_<dart::collision::OdeCollisionDetector, CollisionDetector>(
      m, "OdeCollisionDetector")
      .def(
          nb::new_(
              []() -> std::shared_ptr<dart::collision::OdeCollisionDetector> {
                return dart::collision::OdeCollisionDetector::create();
              }))
      .def_static("getStaticType", []() {
        return std::string(
            dart::collision::OdeCollisionDetector::getStaticType());
      });
#endif
}

} // namespace dart::python_nb
