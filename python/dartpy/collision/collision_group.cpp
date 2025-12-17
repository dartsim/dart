#include "collision/collision_group.hpp"

#include "collision/collision_option.hpp"
#include "collision/collision_result.hpp"
#include "collision/raycast.hpp"
#include "common/eigen_utils.hpp"
#include "common/type_casters.hpp"
#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/ShapeFrame.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defCollisionGroup(nb::module_& m)
{
  using CollisionGroup = dart::collision::CollisionGroup;

  nb::class_<CollisionGroup>(m, "CollisionGroup")
      .def(
          "addShapeFrame",
          [](CollisionGroup& self, const dart::dynamics::ShapeFrame* frame) {
            self.addShapeFrame(frame);
          },
          nb::arg("shape_frame"))
      .def(
          "addShapeFramesOf",
          [](CollisionGroup& self, const dart::dynamics::BodyNode* body) {
            self.addShapeFramesOf(body);
          },
          nb::arg("body"))
      .def("removeAllShapeFrames", &CollisionGroup::removeAllShapeFrames)
      .def("getNumShapeFrames", &CollisionGroup::getNumShapeFrames)
      .def(
          "collide",
          [](CollisionGroup& self,
             const dart::collision::CollisionOption& option,
             dart::collision::CollisionResult* result) {
            return self.getCollisionDetector()->collide(&self, option, result);
          },
          nb::arg("option")
          = dart::collision::CollisionOption(false, 1u, nullptr),
          nb::arg("result") = nullptr)
      .def(
          "collide",
          [](CollisionGroup& self,
             CollisionGroup& other,
             const dart::collision::CollisionOption& option,
             dart::collision::CollisionResult* result) {
            return self.getCollisionDetector()->collide(
                &self, &other, option, result);
          },
          nb::arg("other"),
          nb::arg("option")
          = dart::collision::CollisionOption(false, 1u, nullptr),
          nb::arg("result") = nullptr)
      .def(
          "collideResult",
          [](CollisionGroup& self,
             const dart::collision::CollisionOption& option)
              -> dart::collision::CollisionResult {
            dart::collision::CollisionResult result;
            self.getCollisionDetector()->collide(&self, option, &result);
            return result;
          },
          nb::arg("option") = dart::collision::CollisionOption())
      .def(
          "collideResult",
          [](CollisionGroup& self,
             CollisionGroup& other,
             const dart::collision::CollisionOption& option)
              -> dart::collision::CollisionResult {
            dart::collision::CollisionResult result;
            self.getCollisionDetector()->collide(
                &self, &other, option, &result);
            return result;
          },
          nb::arg("other"),
          nb::arg("option") = dart::collision::CollisionOption())
      .def(
          "raycast",
          [](CollisionGroup& self,
             const nb::handle& from,
             const nb::handle& to,
             const dart::collision::RaycastOption& option,
             dart::collision::RaycastResult* result) {
            return self.getCollisionDetector()->raycast(
                &self, toVector3(from), toVector3(to), option, result);
          },
          nb::arg("from_"),
          nb::arg("to"),
          nb::arg("option") = dart::collision::RaycastOption(),
          nb::arg("result") = nullptr)
      .def(
          "raycastResult",
          [](CollisionGroup& self,
             const nb::handle& from,
             const nb::handle& to,
             const dart::collision::RaycastOption& option)
              -> dart::collision::RaycastResult {
            dart::collision::RaycastResult result;
            self.getCollisionDetector()->raycast(
                &self, toVector3(from), toVector3(to), option, &result);
            return result;
          },
          nb::arg("from_"),
          nb::arg("to"),
          nb::arg("option") = dart::collision::RaycastOption());
}

} // namespace dart::python_nb
