#include "dynamics/simple_frame.hpp"

#include "common/eigen_utils.hpp"
#include "common/repr.hpp"
#include "common/type_casters.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/frame.hpp"
#include "dart/dynamics/simple_frame.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defSimpleFrame(nb::module_& m)
{
  using SimpleFrame = dart::dynamics::SimpleFrame;
  using Frame = dart::dynamics::Frame;

  nb::class_<SimpleFrame, dart::dynamics::ShapeFrame>(m, "SimpleFrame")
      .def(nb::new_([]() { return std::make_shared<SimpleFrame>(); }))
      .def(
          nb::new_([](Frame* refFrame) {
            return std::make_shared<SimpleFrame>(
                refFrame ? refFrame : Frame::World());
          }),
          nb::arg("ref_frame"))
      .def(
          nb::new_([](Frame* refFrame, const std::string& name) {
            return std::make_shared<SimpleFrame>(
                refFrame ? refFrame : Frame::World(), name);
          }),
          nb::arg("ref_frame"),
          nb::arg("name"))
      .def(
          nb::new_([](Frame* refFrame,
                      const std::string& name,
                      const Eigen::Isometry3d& relativeTransform) {
            return std::make_shared<SimpleFrame>(
                refFrame ? refFrame : Frame::World(), name, relativeTransform);
          }),
          nb::arg("ref_frame"),
          nb::arg("name"),
          nb::arg("relative_transform") = Eigen::Isometry3d::Identity())
      .def(
          "isShapeFrame",
          [](const SimpleFrame& self) { return self.isShapeFrame(); })
      .def(
          "isShapeNode",
          [](const SimpleFrame& self) { return self.isShapeNode(); })
      .def("isWorld", [](const SimpleFrame& self) { return self.isWorld(); })
      .def(
          "setName",
          [](SimpleFrame& self, const std::string& name) -> const std::string& {
            return self.setName(name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "__repr__",
          [](const SimpleFrame& self) {
            const auto shape = self.getShape();
            const auto* parent = self.getParentFrame();
            std::vector<std::pair<std::string, std::string>> fields;
            fields.emplace_back("name", repr_string(self.getName()));
            fields.emplace_back(
                "parent", parent ? repr_string(parent->getName()) : "None");
            fields.emplace_back(
                "shape", shape ? repr_string(shape->getType()) : "None");
            return format_repr("SimpleFrame", fields);
          })
      .def(
          "getName",
          [](const SimpleFrame& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "spawnChildSimpleFrame",
          [](SimpleFrame& self) { return self.spawnChildSimpleFrame(); })
      .def(
          "spawnChildSimpleFrame",
          [](SimpleFrame& self, const std::string& name) {
            return self.spawnChildSimpleFrame(name);
          },
          nb::arg("name"))
      .def(
          "spawnChildSimpleFrame",
          [](SimpleFrame& self,
             const std::string& name,
             const Eigen::Isometry3d& relativeTransform) {
            return self.spawnChildSimpleFrame(name, relativeTransform);
          },
          nb::arg("name"),
          nb::arg("relative_transform"))
      .def(
          "setShape",
          [](SimpleFrame& self, const dart::dynamics::ShapePtr& shape) {
            self.setShape(shape);
          },
          nb::arg("shape"))
      .def(
          "getShape",
          [](SimpleFrame& self) -> dart::dynamics::ShapePtr {
            return self.getShape();
          })
      .def(
          "getShape",
          [](const SimpleFrame& self) -> dart::dynamics::ConstShapePtr {
            return self.getShape();
          })
      .def("hasVisualAspect", &SimpleFrame::hasVisualAspect)
      .def(
          "getVisualAspect",
          [](SimpleFrame& self, bool createIfNull) {
            return self.getVisualAspect(createIfNull);
          },
          nb::arg("create_if_null") = false,
          nb::rv_policy::reference_internal)
      .def(
          "createVisualAspect",
          [](SimpleFrame& self) { return self.createVisualAspect(); },
          nb::rv_policy::reference_internal)
      .def("hasCollisionAspect", &SimpleFrame::hasCollisionAspect)
      .def(
          "getCollisionAspect",
          [](SimpleFrame& self, bool createIfNull) {
            return self.getCollisionAspect(createIfNull);
          },
          nb::arg("create_if_null") = false,
          nb::rv_policy::reference_internal)
      .def(
          "createCollisionAspect",
          [](SimpleFrame& self) { return self.createCollisionAspect(); },
          nb::rv_policy::reference_internal)
      .def("hasDynamicsAspect", &SimpleFrame::hasDynamicsAspect)
      .def(
          "getDynamicsAspect",
          [](SimpleFrame& self, bool createIfNull) {
            return self.getDynamicsAspect(createIfNull);
          },
          nb::arg("create_if_null") = true,
          nb::rv_policy::reference_internal)
      .def(
          "createDynamicsAspect",
          [](SimpleFrame& self) { return self.createDynamicsAspect(); },
          nb::rv_policy::reference_internal)
      .def(
          "setRelativeTranslation",
          [](SimpleFrame& self, const nb::handle& translation) {
            self.setRelativeTranslation(toVector3(translation));
          },
          nb::arg("new_translation"))
      .def(
          "setRelativeRotation",
          [](SimpleFrame& self, const Eigen::Matrix3d& rotation) {
            self.setRelativeRotation(rotation);
          },
          nb::arg("new_rotation"))
      .def(
          "setRelativeTransform",
          [](SimpleFrame& self, const Eigen::Isometry3d& transform) {
            self.setRelativeTransform(transform);
          },
          nb::arg("new_rel_transform"))
      .def(
          "setTranslation",
          [](SimpleFrame& self, const nb::handle& translation) {
            self.setTranslation(toVector3(translation));
          },
          nb::arg("new_translation"))
      .def(
          "setTranslation",
          [](SimpleFrame& self,
             const nb::handle& translation,
             const Frame* withRespectTo) {
            const Frame* target
                = withRespectTo ? withRespectTo : Frame::World();
            self.setTranslation(toVector3(translation), target);
          },
          nb::arg("new_translation"),
          nb::arg("with_respect_to"))
      .def(
          "setTransform",
          [](SimpleFrame& self, const Eigen::Isometry3d& transform) {
            self.setTransform(transform);
          },
          nb::arg("new_transform"))
      .def(
          "setTransform",
          [](SimpleFrame& self,
             const Eigen::Isometry3d& transform,
             const Frame* withRespectTo) {
            const Frame* target
                = withRespectTo ? withRespectTo : Frame::World();
            self.setTransform(transform, target);
          },
          nb::arg("new_transform"),
          nb::arg("with_respect_to"))
      .def(
          "getTransform",
          [](const SimpleFrame& self) { return self.getTransform(); })
      .def(
          "getTransform",
          [](const SimpleFrame& self, const Frame* withRespectTo) {
            const Frame* target
                = withRespectTo ? withRespectTo : Frame::World();
            return self.getTransform(target);
          },
          nb::arg("with_respect_to"))
      .def(
          "getParentFrame",
          [](SimpleFrame& self) -> Frame* { return self.getParentFrame(); },
          nb::rv_policy::reference)
      .def(
          "descendsFrom",
          [](const SimpleFrame& self, const Frame* someFrame) {
            return self.descendsFrom(someFrame);
          },
          nb::arg("some_frame") = nullptr);

  registerPolymorphicCaster<Frame, SimpleFrame>();
  registerPolymorphicCaster<dart::dynamics::ShapeFrame, SimpleFrame>();
}

} // namespace dart::python_nb
