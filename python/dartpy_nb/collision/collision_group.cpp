#include "collision/collision_group.hpp"

#include "collision/collision_option.hpp"
#include "collision/collision_result.hpp"
#include "collision/raycast.hpp"
#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/ShapeFrame.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "common/eigen_utils.hpp"
#include "common/type_casters.hpp"

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
          nb::arg("shapeFrame"))
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
          "raycast",
          [](CollisionGroup& self,
             const nb::handle& from,
             const nb::handle& to,
             const dart::collision::RaycastOption& option,
             dart::collision::RaycastResult* result) {
            return self.getCollisionDetector()->raycast(
                &self, toVector3(from), toVector3(to), option, result);
          },
          nb::arg("from"),
          nb::arg("to"),
          nb::arg("option") = dart::collision::RaycastOption(),
          nb::arg("result") = nullptr);
}

} // namespace dart::python_nb
